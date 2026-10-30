#include "car.h"
#include <stdio.h>

float Velocity_KP = 6.5f;
float Velocity_KI = 1.33f;
QueueHandle_t xVofaQueue = NULL;
static inline float float_abs(float insert)
{
    return (insert >= 0.0f) ? insert : -insert;
}

float target_speed = 0.0f;

float target_speed_l = 0.0f;
float target_speed_r = 0.0f;
float actual_speed_l = 0.0f;
float actual_speed_r = 0.0f;

int encoder_l = 0;
int encoder_r = 0;

int current_pwm_l = 0;
int current_pwm_r = 0;
int target_servo = 1500;

uint8_t car_is_armed = 0;
uint8_t car_gear = 2;
uint8_t ps2_is_online = 0;

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

        /* 第 3 行 (Y=32): 左轮脉冲 L + 目标脉冲 T */
        OLED_ShowString(0, 32, (u8 *)"L:");
        ShowSignedNum(12, 32, encoder_l, 4, 12);
        OLED_ShowString(48, 32, (u8 *)" T:");
        ShowSignedNum(66, 32, (int)target_speed, 4, 12);

        /* 第 4 行 (Y=48): 右轮脉冲 R + 舵机脉宽 S */
        OLED_ShowString(0, 48, (u8 *)"R:");
        ShowSignedNum(12, 48, encoder_r, 4, 12);
        OLED_ShowString(48, 48, (u8 *)" S:");
        OLED_ShowNumber(66, 48, (uint32_t)target_servo, 4, 12);

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
    float ratio = 0.0f;
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
            target_speed = 0.0f;
            target_speed_l = 0.0f;
            target_speed_r = 0.0f;

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

        /* 3. 计算目标速度 (以编码器脉冲/10ms 为单位) 与阿克曼转向差速 */
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

            /* 1档: 300脉冲, 2档: 600脉冲, 3档: 900脉冲 */
            float max_gear_pulses = (car_gear == 1) ? 300.0f :
                                    (car_gear == 2) ? 600.0f : 900.0f;
            target_speed = norm_throttle * max_gear_pulses;

            /* 右摇杆 X 轴与十字键左右控制前轮舵机转向 (1100 ~ 1900us, 中位 1500us) */
            int rx = (int)PS2_RX;
            int steer = 0;
            if (Data[1] == 0x73 || Data[1] == 0x79)
            {
                if (rx < 115)      steer = (rx - 115);
                else if (rx > 140) steer = (rx - 140);
            }
            if (cur_key == PSB_PAD_LEFT)       steer = -115;
            else if (cur_key == PSB_PAD_RIGHT) steer = 115;

            target_servo = SERVO_INIT + (int)((float)steer * 400.0f / 115.0f);
            if (target_servo < 1100) target_servo = 1100;
            if (target_servo > 1900) target_servo = 1900;

            /* 阿克曼转向内轮减速、外轮加速差速分配 */
            if (target_servo < SERVO_INIT)
            {
                ratio = (float)(SERVO_INIT - target_servo) / 400.0f;
                target_speed_l = target_speed * (1.0f - ratio * 0.35f);
                target_speed_r = target_speed * (1.0f + ratio * 0.35f);
            }
            else if (target_servo > SERVO_INIT)
            {
                ratio = (float)(target_servo - SERVO_INIT) / 400.0f;
                target_speed_l = target_speed * (1.0f + ratio * 0.35f);
                target_speed_r = target_speed * (1.0f - ratio * 0.35f);
            }
            else
            {
                target_speed_l = target_speed;
                target_speed_r = target_speed;
            }
        }
        else if (!car_is_armed)
        {
            target_speed = 0.0f;
            target_speed_l = 0.0f;
            target_speed_r = 0.0f;
            target_servo = SERVO_INIT;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

