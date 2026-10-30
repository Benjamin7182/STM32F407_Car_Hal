#include "motor.h"

// 启动四路电机与舵机的 PWM 输出
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); // Motor A
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_1); // Motor B
    HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_1); // Motor C & D
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_4);

    // 启动前轮转向舵机 TIM12_CH2 (PB15)
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    Servo_PWM = SERVO_INIT; // 上电默认舵机居中 (1500us)
}

// 赋值 PWM 寄存器控制电机转速与转向舵机
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo)
{
    // Motor A
    if (motor_a < 0) {
        PWMA1 = 16799;
        PWMA2 = 16799 + motor_a;
    } else {
        PWMA2 = 16799;
        PWMA1 = 16799 - motor_a;
    }

    // Motor B
    if (motor_b < 0) {
        PWMB1 = 16799;
        PWMB2 = 16799 + motor_b;
    } else {
        PWMB2 = 16799;
        PWMB1 = 16799 - motor_b;
    }

    // Motor C
    if (motor_c < 0) {
        PWMC1 = 16799;
        PWMC2 = 16799 + motor_c;
    } else {
        PWMC2 = 16799;
        PWMC1 = 16799 - motor_c;
    }

    // Motor D
    if (motor_d < 0) {
        PWMD1 = 16799;
        PWMD2 = 16799 + motor_d;
    } else {
        PWMD2 = 16799;
        PWMD1 = 16799 - motor_d;
    }

    // 舵机角度输出 (带安全限幅 800 ~ 2200us)
    Set_Servo(servo);
}

// 独立设置舵机 PWM 脉宽 (微秒)，带安全限幅
void Set_Servo(int servo_val)
{
    if (servo_val < 800)  servo_val = 800;
    if (servo_val > 2200) servo_val = 2200;
    Servo_PWM = servo_val;
}

// 电机全部停机制动
void Motor_Stop(void)
{
    PWMA1 = 0; PWMA2 = 0;
    PWMB1 = 0; PWMB2 = 0;
    PWMC1 = 0; PWMC2 = 0;
    PWMD1 = 0; PWMD2 = 0;
}
