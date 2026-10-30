#include "car.h"
#include <stdio.h>
#include <math.h>

float Velocity_KP = 6.5f;
float Velocity_KI = 1.33f;
QueueHandle_t xVofaQueue = NULL;

static inline float float_abs(float insert)
{
    return (insert >= 0.0f) ? insert : -insert;
}

float target_speed = 0.0f;

/* 闭环 PI 目标速度 (单位: 编码器脉冲/10ms) */
float target_speed_l = 0.0f;
float target_speed_r = 0.0f;

/* 正运动学解算出的实际物理速度 */
float actual_speed_l = 0.0f;      /* 左后轮实际线速度 (m/s) */
float actual_speed_r = 0.0f;      /* 右后轮实际线速度 (m/s) */
float chassis_vx = 0.0f;          /* 车体中心前向线速度 Vx (m/s) */
float chassis_vz = 0.0f;          /* 车体中心偏航角速度 Vz (rad/s) */
int16_t chassis_vx_mms = 0;       /* 车体前向线速度 (mm/s, 对应 ROS X_speed) */
int16_t chassis_vy_mms = 0;       /* 车体横向线速度 (mm/s, 阿克曼恒为 0) */
int16_t chassis_vz_mrads = 0;     /* 车体偏航角速度 (mrad/s, 对应 ROS Z_speed) */

int encoder_l = 0;
int encoder_r = 0;

int current_pwm_l = 0;
int current_pwm_r = 0;
int target_servo = SERVO_INIT;

uint8_t car_is_armed = 0;
uint8_t car_gear = 2;
uint8_t ps2_is_online = 0;

/* ============================================================================
 * 1. 阿克曼正运动学解算 (STM32 编码器脉冲 -> 车体物理速度 Vx, Vz -> ROS Odom)
 * ============================================================================ */
void Kinematics_Forward(void)
{
    /* 1. 左右后轮编码器脉冲 (pulses/10ms) -> 轮线速度 (m/s) */
    actual_speed_l = (float)encoder_l * PULSE_TO_SPEED_MS;
    actual_speed_r = (float)encoder_r * PULSE_TO_SPEED_MS;

    /* 2. 双后轮差速模型求车体中心前向线速度 Vx (m/s) 与偏航角速度 Vz (rad/s) */
    chassis_vx = (actual_speed_l + actual_speed_r) * 0.5f;
    chassis_vz = (actual_speed_r - actual_speed_l) / AKM_WHEEL_SPACING;

    /* 3. 转换为 ROS 串口协议整数单位: mm/s 与 mrad/s */
    chassis_vx_mms   = (int16_t)(chassis_vx * 1000.0f);
    chassis_vy_mms   = 0;
    chassis_vz_mrads = (int16_t)(chassis_vz * 1000.0f);
}

/* ============================================================================
 * 2. 阿克曼逆运动学解算 (目标 Vx, Vz / 前轮转角 -> 左右后轮目标脉冲 + 舵机 PWM)
 * ============================================================================ */

/* 将目标前向线速度 Vx (m/s) 与目标角速度 Vz (rad/s) 换算为阿克曼前轮转角 (rad) */
float Vz_to_Akm_Angle(float vx_ms, float vz_rads)
{
    float r_turn, angle_r;

    if (vz_rads == 0.0f || vx_ms == 0.0f)
    {
        return 0.0f;
    }

    /* 限制最小转弯半径 R_min = 0.350m */
    if (float_abs(vx_ms / vz_rads) < AKM_MIN_TURN_RADIUS)
    {
        vz_rads = vx_ms / (AKM_MIN_TURN_RADIUS * (vz_rads >= 0.0f ? 1.0f : -1.0f));
    }

    r_turn = vx_ms / vz_rads;
    angle_r = atanf(AKM_AXLE_SPACING / (r_turn + 0.5f * AKM_WHEEL_SPACING));

    return angle_r;
}

/* 已知目标车体线速度 vx_ms (m/s) 和前轮转角 angle_r_rad (rad)，解算后轮差速与舵机连杆 PWM */
void Kinematics_Inverse_Angle(float vx_ms, float angle_r_rad)
{
    float r_turn;
    float vel_l_ms, vel_r_ms;
    float angle_servo;

    /* 1. 机械前轮转角限幅 (对齐原厂限幅 [-0.49, +0.32] rad) */
    if (angle_r_rad > 0.32f)  angle_r_rad = 0.32f;
    if (angle_r_rad < -0.49f) angle_r_rad = -0.49f;

    /* 2. 阿克曼几何差速公式计算左右后轮目标线速度 (m/s) */
    if (float_abs(angle_r_rad) > 0.001f)
    {
        r_turn = AKM_AXLE_SPACING / tanf(angle_r_rad) - 0.5f * AKM_WHEEL_SPACING;
        vel_l_ms = vx_ms * (r_turn - 0.5f * AKM_WHEEL_SPACING) / r_turn;
        vel_r_ms = vx_ms * (r_turn + 0.5f * AKM_WHEEL_SPACING) / r_turn;
    }
    else
    {
        vel_l_ms = vx_ms;
        vel_r_ms = vx_ms;
    }

    /* 3. 将目标轮速 (m/s) 换算为编码器目标脉冲 (pulses/10ms) 供增量式 PI 控制器跟踪 */
    target_speed   = vx_ms    * SPEED_MS_TO_PULSE;
    target_speed_l = vel_l_ms * SPEED_MS_TO_PULSE;
    target_speed_r = vel_r_ms * SPEED_MS_TO_PULSE;

    /* 4. 阿克曼转向连杆三次多项式非线性补偿 -> 舵机 PWM 脉宽 (us) */
    angle_servo = -0.628f * powf(angle_r_rad, 3.0f)
                  + 1.269f * powf(angle_r_rad, 2.0f)
                  - 1.772f * angle_r_rad
                  + 1.573f;

    target_servo = SERVO_INIT + (int)((angle_servo - 1.572f) * 636.56f);
    if (target_servo < 1050) target_servo = 1050;
    if (target_servo > 1950) target_servo = 1950;
}

/* ROS /cmd_vel 标准入口: 输入目标线速度 vx_ms (m/s) 与角速度 vz_rads (rad/s) */
void Kinematics_Inverse_Vz(float vx_ms, float vz_rads)
{
    float angle_r = Vz_to_Akm_Angle(vx_ms, vz_rads);
    Kinematics_Inverse_Angle(vx_ms, angle_r);
}

static void ShowSignedNum(u8 x, u8 y, int val, u8 len, u8 size)
{
    if (val < 0)
    {
        OLED_ShowChar(x, y, '-', size, 1);
        OLED_ShowNumber(x + size / 2, y, (uint32_t)(-val), len, size);
    }
    else
    {
        OLED_ShowChar(x, y, '+', size, 1);
        OLED_ShowNumber(x + size / 2, y, (uint32_t)val, len, size);
    }
}

void OledTask(void *pvParameters)
{
    while (1)
    {
        float volt = Get_Battery_Voltage();

        /* 第 1 行 (Y=0): 状态 + 电池电压 */
        if (Deviation_Count < CONTROL_DELAY)
        {
            OLED_ShowString(0, 0, (u8 *)"[IMU CAL..] ");
        }
        else if (!ps2_is_online)
        {
            OLED_ShowString(0, 0, (u8 *)"[NO PS2]    ");
        }
        else if (!car_is_armed)
        {
            OLED_ShowString(0, 0, (u8 *)"[LOCK ST]   ");
        }
        else
        {
            if (car_gear == 1)      OLED_ShowString(0, 0, (u8 *)"[ARM G1]    ");
            else if (car_gear == 2) OLED_ShowString(0, 0, (u8 *)"[ARM G2]    ");
            else                    OLED_ShowString(0, 0, (u8 *)"[ARM G3]    ");
        }
        OLED_ShowFloat(72, 0, volt, 2, 1, 12);
        OLED_ShowChar(102, 0, 'V', 12, 1);

        /* 第 2 行 (Y=16): MPU6050 偏航角 YAW (度) + Z轴角速度 GZ */
        OLED_ShowString(0, 16, (u8 *)"YAW:");
        ShowSignedNum(24, 16, (int)Yaw, 3, 12);
        OLED_ShowString(54, 16, (u8 *)" GZ:");
        ShowSignedNum(78, 16, (int)gyro[2], 5, 12);

        /* 第 3 行 (Y=32): 左轮脉冲 L + 右轮脉冲 R */
        OLED_ShowString(0, 32, (u8 *)"L:");
        ShowSignedNum(12, 32, encoder_l, 4, 12);
        OLED_ShowString(54, 32, (u8 *)" R:");
        ShowSignedNum(72, 32, encoder_r, 4, 12);

        /* 第 4 行 (Y=48): 正运动学车体速度 VX (mm/s) + 舵机脉宽 S (us) */
        OLED_ShowString(0, 48, (u8 *)"VX:");
        ShowSignedNum(18, 48, (int)chassis_vx_mms, 4, 12);
        OLED_ShowString(54, 48, (u8 *)" S:");
        OLED_ShowNumber(72, 48, (uint32_t)target_servo, 4, 12);

        OLED_Refresh_Gram();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void PID_task(void *pvParameters)
{
    VofaPacket_t tx_pkt;

    /* 蜂鸣器开机提示音 */
    HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(80));
    HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);

    Incremental_PI_Clear();
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        encoder_l = Read_Encoder(2);
        encoder_r = -Read_Encoder(3);

        /* 正运动学解算: 实时计算左右轮 m/s 线速度及底盘 Vx (mm/s), Vz (mrad/s) */
        Kinematics_Forward();

        /* 停车/上锁死区保护：目标为 0 且车轮已停转时清零积分，防止电机静止微鸣 */
        if (!car_is_armed ||
            (float_abs(target_speed_l) < 1.0f && float_abs(target_speed_r) < 1.0f &&
             encoder_l > -5 && encoder_l < 5 && encoder_r > -5 && encoder_r < 5))
        {
            Incremental_PI_Clear();
            current_pwm_l = 0;
            current_pwm_r = 0;
        }
        else
        {
            /* 闭环增量式 PI 控制：以编码器脉冲数为单位跟踪 target_speed_l / target_speed_r */
            current_pwm_l = Incremental_PI_A((float)encoder_l, target_speed_l);
            current_pwm_r = Incremental_PI_B((float)encoder_r, target_speed_r);
        }

        Set_Pwm(current_pwm_l, current_pwm_r, 0, 0, target_servo);

        /* 入队发送到 VOFA+ */
        tx_pkt.target_speed_l = target_speed_l;
        tx_pkt.encl = (float)encoder_l;
        tx_pkt.target_speed_r = target_speed_r;
        tx_pkt.encr = (float)encoder_r;

        if (xVofaQueue != NULL)
        {
            xQueueSend(xVofaQueue, &tx_pkt, 0);
        }
    }
}

void VofaTask(void *pvParameters)
{
    VofaPacket_t rx_pkt;
    char csv_buf[48];
    while (1)
    {
        if (xQueueReceive(xVofaQueue, &rx_pkt, portMAX_DELAY) == pdPASS)
        {
            /* FireWater 协议: "左轮目标,左轮实际,右轮实际\n" */
            int len = snprintf(csv_buf, sizeof(csv_buf), "%d,%d,%d\n",
                               (int)rx_pkt.target_speed_l, (int)rx_pkt.encl, (int)rx_pkt.encr);
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)csv_buf, (uint16_t)len, 10);
            }
        }
    }
}

void Car_Remote_Task(void *pvParameters)
{
    uint8_t last_key = 0;
    uint8_t cfg_counter = 0;

    while (1)
    {
        /* 1. 读取 PS2 遥控手柄 (50Hz 采样) */
        PS2_Read();

        /* PS2 在线检测 */
        ps2_is_online = ((Data[1] == 0x73 || Data[1] == 0x79 || Data[1] == 0x41 ||
                          Data[1] == 0x53 || Data[1] == 0xF3 || Data[2] == 0x5A) &&
                         (Data[1] != 0xFF && Data[1] != 0x00));

        /* 绿灯数字模式自适应尝试切换模拟红灯 */
        if (ps2_is_online && Data[1] == 0x41)
        {
            if (++cfg_counter > 50)
            {
                cfg_counter = 0;
                PS2_SetInit();
            }
        }

        /* 2. 按键边缘检测 */
        uint8_t cur_key = (uint8_t)PS2_KEY;
        uint8_t key_down = (cur_key != last_key) ? cur_key : 0;
        last_key = cur_key;

        /* START 键: 解锁 / 上锁安全切换 */
        if (key_down == PSB_START && ps2_is_online)
        {
            car_is_armed = !car_is_armed;
            Kinematics_Inverse_Angle(0.0f, 0.0f);

            if (car_is_armed)
            {
                HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(40));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
                vTaskDelay(pdMS_TO_TICKS(40));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(40));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(150));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            }
        }

        /* 档位切换: L1 升档, L2 降档 */
        if (key_down == PSB_L1 && ps2_is_online)
        {
            if (car_gear < 3)
            {
                car_gear++;
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(30));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            }
        }
        if (key_down == PSB_L2 && ps2_is_online)
        {
            if (car_gear > 1)
            {
                car_gear--;
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(30));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            }
        }

        /* 鸣笛: 长按 R1 或三角形按键 */
        if (cur_key == PSB_R1 || cur_key == PSB_TRIANGLE)
        {
            HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
        }
        else if (!car_is_armed || cur_key == 0)
        {
            HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
        }

        /* 3. 通过阿克曼逆运动学解算目标速度 (m/s) 与前轮转角 (rad) */
        if (ps2_is_online && car_is_armed)
        {
            /* 左摇杆 Y 轴与十字键上下控制前后油门 (-1.0 ~ +1.0) */
            int ly = (int)PS2_LY;
            float norm_throttle = 0.0f;
            if (Data[1] == 0x73 || Data[1] == 0x79)
            {
                if (ly < 115)      norm_throttle = (float)(115 - ly) / 115.0f;
                else if (ly > 140) norm_throttle = -(float)(ly - 140) / 115.0f;
            }
            if (cur_key == PSB_PAD_UP)        norm_throttle = 1.0f;
            else if (cur_key == PSB_PAD_DOWN) norm_throttle = -1.0f;

            /* 档位对应最大物理线速度 (m/s): 1档 0.10m/s, 2档 0.20m/s, 3档 0.35m/s */
            float max_gear_speed_ms = (car_gear == 1) ? GEAR_1_MAX_SPEED :
                                      (car_gear == 2) ? GEAR_2_MAX_SPEED : GEAR_3_MAX_SPEED;
            float target_vx_ms = norm_throttle * max_gear_speed_ms;

            /* 右摇杆 X 轴与十字键左右控制前轮转角 (左转为正 +0.32 rad, 右转为负 -0.45 rad) */
            int rx = (int)PS2_RX;
            float norm_steer = 0.0f; /* 左推为 +1.0, 右推为 -1.0 */
            if (Data[1] == 0x73 || Data[1] == 0x79)
            {
                if (rx < 115)      norm_steer = (float)(115 - rx) / 115.0f;
                else if (rx > 140) norm_steer = -(float)(rx - 140) / 115.0f;
            }
            if (cur_key == PSB_PAD_LEFT)       norm_steer = 1.0f;
            else if (cur_key == PSB_PAD_RIGHT) norm_steer = -1.0f;

            float target_angle_r = (norm_steer >= 0.0f) ? (norm_steer * 0.32f)
                                                        : (norm_steer * 0.45f);

            /* 调用阿克曼逆运动学：自动计算左右后轮差速目标脉冲与连杆补偿后的舵机 PWM */
            Kinematics_Inverse_Angle(target_vx_ms, target_angle_r);
        }
        else if (!car_is_armed)
        {
            Kinematics_Inverse_Angle(0.0f, 0.0f);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


