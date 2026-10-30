#include "car.h"
#include <stdio.h>

float Velocity_KP = 300.0f;
float Velocity_KI = 300.0f;
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
int current_pwm_l = 1500;
int current_pwm_r = 0;
int target_servo = 1500;
void OledTask(void *pvParameters){
while(1){
			OLED_ShowNumber(18, 48, (encoder_l), 4, 12);
//			OLED_ShowNumber(18, 0, (uint32_t)(encoder_r), 4, 12);
	
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

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        encoder_l = Read_Encoder(2);
        encoder_r = -Read_Encoder(3);

//        current_pwm_l = Incremental_PI_A(actual_speed_l, target_speed_l);
//        current_pwm_r = Incremental_PI_B(actual_speed_r, target_speed_r);
        Set_Pwm(current_pwm_l, 0, 0, 0, target_servo);

        tx_pkt.enc_l = (float)encoder_l;
        tx_pkt.pwm_l = (float)current_pwm_l;
        if (xVofaQueue != NULL)
        {
            xQueueSend(xVofaQueue, &tx_pkt, 0);
        }
    }
}

void VofaTask(void *pvParameters)
{
    VofaPacket_t rx_pkt;
    char csv_buf[32];
    while (1)
    {
        if (xQueueReceive(xVofaQueue, &rx_pkt, portMAX_DELAY) == pdPASS)
        {
            /* FireWater 协议 (标准 CSV 格式: "PWM,编码器脉冲\n") */
            int len = snprintf(csv_buf, sizeof(csv_buf), "%d,%d\n", (int)rx_pkt.pwm_l, (int)rx_pkt.enc_l);
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)csv_buf, (uint16_t)len, 10);
            }
        }
    }
}
void Car_Remote_Task(void *pvParameters)
{
    uint8_t is_armed = 0;
    uint8_t gear = 2; // 默认 2 档 (0.50 m/s，对齐原厂 RC_Velocity = 500)
    uint8_t last_key = 0;
    uint8_t oled_divider = 0;
    uint8_t vofa_divider = 0;
    float ratio = 0.0f;
    uint8_t cfg_counter = 0;

    /* 清空 PI 初始累积 */
    Incremental_PI_Clear();

    while (1)
    {
        /* 1. 读取 PS2 遥控手柄 */
        PS2_Read();

        /* PS2 在线检测 */
        uint8_t ps2_online = ((Data[1] == 0x73 || Data[1] == 0x79 || Data[1] == 0x41 || 
                               Data[1] == 0x53 || Data[1] == 0xF3 || Data[2] == 0x5A) && 
                              (Data[1] != 0xFF && Data[1] != 0x00));

        /* 绿灯数字模式自适应尝试切换模拟红灯 */
        if (ps2_online && Data[1] == 0x41) {
            if (++cfg_counter > 100) {
                cfg_counter = 0;
                PS2_SetInit();
            }
        }

        /* 2. 读取传感器并转化为原厂标准线速度单位 (m/s) */
        float volt = Get_Battery_Voltage();
        encoder_l = Read_Encoder(2);
        encoder_r = -Read_Encoder(3);
        actual_speed_l = (float)encoder_l * PULSE_TO_SPEED_MS;
        actual_speed_r = (float)encoder_r * PULSE_TO_SPEED_MS;

        /* 3. 按键边缘检测 */
        uint8_t cur_key = (uint8_t)PS2_KEY;
        uint8_t key_down = (cur_key != last_key) ? cur_key : 0;
        last_key = cur_key;

        /* START 键: 解锁 / 上锁安全切换 */
        if (key_down == PSB_START && ps2_online)
        {
            is_armed = !is_armed;
            Incremental_PI_Clear();
            target_speed = 0.0f;

            if (is_armed) {
                HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(40));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
                vTaskDelay(pdMS_TO_TICKS(40));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(40));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(150));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            }
        }

        /* 档位切换: L1 升档, L2 降档 */
        if (key_down == PSB_L1 && ps2_online) {
            if (gear < 3) {
                gear++;
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(30));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            }
        }
        if (key_down == PSB_L2 && ps2_online) {
            if (gear > 1) {
                gear--;
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
                vTaskDelay(pdMS_TO_TICKS(30));
                HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
            }
        }

        /* 鸣笛: 长按 R1 或三角形按键 */
        if (cur_key == PSB_R1 || cur_key == PSB_TRIANGLE) {
            HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
        } else if (!is_armed || cur_key == 0) {
            HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
        }

        /* 4. 电池低压保护 (<10V 强制失能停车) */
        if (volt < 10.0f && volt > 1.0f) {
            is_armed = 0;
        }

        /* 5. 闭环速度控制与阿克曼差速解算 (100Hz 对齐原厂 Balance_task) */
        if (ps2_online && is_armed)
        {
            /* 摇杆归一化油门 (-1.0 ~ +1.0)，支持左摇杆与十字键 */
            int ly = (int)PS2_LY;
            float norm_throttle = 0.0f;
            if (ly < 115) {
                norm_throttle = (float)(115 - ly) / 115.0f;
            } else if (ly > 140) {
                norm_throttle = -(float)(ly - 140) / 115.0f;
            } else if (cur_key == PSB_PAD_UP) {
                norm_throttle = 1.0f;
            } else if (cur_key == PSB_PAD_DOWN) {
                norm_throttle = -1.0f;
            }

            /* 当前档位最大速度 (m/s) */
            float max_gear_speed = (gear == 1) ? GEAR_1_MAX_SPEED : 
                                   (gear == 2) ? GEAR_2_MAX_SPEED : GEAR_3_MAX_SPEED;
            float desired_speed = norm_throttle * max_gear_speed;
            
            /* 目标线速度直接赋值 (对齐原厂 Akm_Car: MOTOR.Target = Vx) */
            target_speed = desired_speed;

            /* 右摇杆控制前轮舵机转向: 0~255, 中位 ~128 */
            int rx = (int)PS2_RX;
            int steer = 0;
            if (rx < 115) {
                steer = (rx - 115);
            } else if (rx > 140) {
                steer = (rx - 140);
            } else if (cur_key == PSB_PAD_LEFT) {
                steer = -115;
            } else if (cur_key == PSB_PAD_RIGHT) {
                steer = 115;
            }

            target_servo = SERVO_INIT + (int)((float)steer * 400.0f / 115.0f);
            if (target_servo < 1100) target_servo = 1100;
            if (target_servo > 1900) target_servo = 1900;

            /* 阿克曼差速几何分配 */
            if (target_servo < SERVO_INIT) {
                ratio = (float)(SERVO_INIT - target_servo) / 400.0f;
                target_speed_l = target_speed * (1.0f - ratio * 0.35f);
                target_speed_r = target_speed * (1.0f + ratio * 0.35f);
            } else if (target_servo > SERVO_INIT) {
                ratio = (float)(target_servo - SERVO_INIT) / 400.0f;
                target_speed_l = target_speed * (1.0f + ratio * 0.35f);
                target_speed_r = target_speed * (1.0f - ratio * 0.35f);
            } else {
                target_speed_l = target_speed;
                target_speed_r = target_speed;
            }

            /* 停车静止死区保护：遥控回中且小车停止时，复位 PI 消除电磁微鸣 */
            if (desired_speed == 0.0f && (float_abs(target_speed) < 0.02f) &&
                (float_abs(actual_speed_l) < 0.03f) && (float_abs(actual_speed_r) < 0.03f))
            {
                Incremental_PI_Clear();
                current_pwm_l = 0;
                current_pwm_r = 0;
            }
            else
            {
                /* 原标准库 balance.c 原版增量式 PI 控制器计算 (输入 m/s，输出 PWM) */
                current_pwm_l = Incremental_PI_A(actual_speed_l, target_speed_l);
                current_pwm_r = Incremental_PI_B(actual_speed_r, target_speed_r);
            }

            Set_Pwm(current_pwm_l, current_pwm_r, 0, 0, target_servo);
        }
        else
        {
            target_speed = 0.0f;
            target_speed_l = 0.0f;
            target_speed_r = 0.0f;
            current_pwm_l = 0;
            current_pwm_r = 0;
            target_servo = SERVO_INIT;
            Incremental_PI_Clear();
            Set_Pwm(0, 0, 0, 0, SERVO_INIT);
            Motor_Stop();
        }

        /* 6. OLED 实时仪表盘 (100ms 刷新一次，对齐原厂 show.c) */
        oled_divider++;
        if (oled_divider >= 10)
        {
            oled_divider = 0;

            if (!ps2_online) {
                OLED_ShowString(0, 0, (u8*)"[NO PS2] ID:");
                OLED_ShowNumber(72, 0, (uint32_t)Data[1], 3, 12);
                OLED_ShowChar(92, 0, ':', 12, 1);
                OLED_ShowNumber(98, 0, (uint32_t)Data[2], 3, 12);
            } else if (!is_armed) {
                OLED_ShowString(0, 0, (u8*)"[LOCK] PRESS ST ");
            } else {
                if (gear == 1)      OLED_ShowString(0, 0, (u8*)"[ARMED] G:1(LOW)");
                else if (gear == 2) OLED_ShowString(0, 0, (u8*)"[ARMED] G:2(MID)");
                else                OLED_ShowString(0, 0, (u8*)"[ARMED] G:3(HI) ");
            }

            OLED_ShowString(0, 16, (u8*)"VOLT:");
            OLED_ShowFloat(40, 16, volt, 2, 2, 12);
            if (volt < 1.0f) {
                OLED_ShowString(76, 16, (u8*)"V USB   ");
            } else if (volt < 10.0f) {
                OLED_ShowString(76, 16, (u8*)"V LOW!! ");
            } else {
                OLED_ShowString(76, 16, (u8*)"V (3S)  ");
            }

            // 第 2 行：目标速度 T (* 1000 显示 mm/s) 与舵机脉宽 S (us)
            OLED_ShowString(0, 32, (u8*)"T:");
            int disp_tgt = (int)(target_speed * 1000.0f);
            if (disp_tgt < 0) {
                OLED_ShowChar(14, 32, '-', 12, 1);
                OLED_ShowNumber(20, 32, (uint32_t)(-disp_tgt), 4, 12);
            } else {
                OLED_ShowChar(14, 32, '+', 12, 1);
                OLED_ShowNumber(20, 32, (uint32_t)disp_tgt, 4, 12);
            }
            OLED_ShowString(48, 32, (u8*)" S:");
            OLED_ShowNumber(64, 32, (uint32_t)target_servo, 4, 12);
            OLED_ShowString(90, 32, (u8*)"us  ");

            // 第 3 行：实测左右轮车速 L/R (* 1000 显示 mm/s，对齐原厂 show.c)
            OLED_ShowString(0, 48, (u8*)"L:");
            int disp_l = (int)(actual_speed_l * 1000.0f);
            if (disp_l < 0) {
                OLED_ShowChar(12, 48, '-', 12, 1);
                OLED_ShowNumber(18, 48, (uint32_t)(-disp_l), 4, 12);
            } else {
                OLED_ShowChar(12, 48, '+', 12, 1);
                OLED_ShowNumber(18, 48, (uint32_t)disp_l, 4, 12);
            }

            OLED_ShowString(46, 48, (u8*)"R:");
            int disp_r = (int)(actual_speed_r * 1000.0f);
            if (disp_r < 0) {
                OLED_ShowChar(58, 48, '-', 12, 1);
                OLED_ShowNumber(64, 48, (uint32_t)(-disp_r), 4, 12);
            } else {
                OLED_ShowChar(58, 48, '+', 12, 1);
                OLED_ShowNumber(64, 48, (uint32_t)disp_r, 4, 12);
            }
            OLED_ShowString(90, 48, (u8*)"mm/s");

            OLED_Refresh_Gram();
        }

        /* 7. VOFA+ 实时波形发送 (调用 APP/vofa.c 的 JustFloat 驱动，每 20ms 发送一次) */
        vofa_divider++;
        if (vofa_divider >= 2)
        {
            vofa_divider = 0;
            /* 轮子转速 RPM (转/分钟): RPM = (线速度 m/s / 周长 0.2042m) * 60 = 速度 * 293.82f */
            float target_rpm   = target_speed * 293.82f;
            float actual_l_rpm = actual_speed_l * 293.82f;
            float actual_r_rpm = actual_speed_r * 293.82f;
            VOFA_Send_Speed(&huart1, target_rpm, actual_l_rpm, actual_r_rpm);
        }

        /* 10ms 控制周期 (100Hz，完全对齐原厂 FreeRTOS Balance_task RATE_100_HZ) */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

