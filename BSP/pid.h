#ifndef __PID_H
#define __PID_H

#include "main.h"

/* 500线 GMR 编码器 + 1:30 减速比 + 4倍频 = 60000 脉冲/圈, 轮径 D = 0.065m, 采样周期 Ts = 10ms (100Hz)
 * 1 pulse/10ms = (pi * 0.065 * 100) / 60000 = 0.000340339 m/s
 * 1 m/s        = 60000 / (pi * 0.065 * 100) = 2938.24 pulses/10ms
 */
#define PULSE_TO_SPEED_MS   0.000340339f
#define SPEED_MS_TO_PULSE   2938.24f

extern float Velocity_KP;
extern float Velocity_KI;

/* 增量式 PI 速度控制器 A (左轮 / 电机 1) */
static inline int Incremental_PI_A(float actual_speed, float target_speed)
{
    static float bias = 0.0f;
    static float last_bias = 0.0f;
    static int pwm = 0;

    if (target_speed == 0.0f && actual_speed == 0.0f)
    {
        bias = 0.0f;
        last_bias = 0.0f;
        pwm = 0;
        return 0;
    }

    bias = target_speed - actual_speed;

    // 增量式 PI 离散公式: dPWM = KP * (e_k - e_{k-1}) + KI * e_k
    pwm += (int)(Velocity_KP * (bias - last_bias) + Velocity_KI * bias);

    last_bias = bias;

    // 输出限幅在 STM32 PWM 定时器满幅范围内 [-7200, +7200]
    if (pwm > 7200)  pwm = 7200;
    if (pwm < -7200) pwm = -7200;

    return pwm;
}

/* 增量式 PI 速度控制器 B (右轮 / 电机 2) */
static inline int Incremental_PI_B(float actual_speed, float target_speed)
{
    static float bias = 0.0f;
    static float last_bias = 0.0f;
    static int pwm = 0;

    if (target_speed == 0.0f && actual_speed == 0.0f)
    {
        bias = 0.0f;
        last_bias = 0.0f;
        pwm = 0;
        return 0;
    }

    bias = target_speed - actual_speed;

    // 增量式 PI 离散公式: dPWM = KP * (e_k - e_{k-1}) + KI * e_k
    pwm += (int)(Velocity_KP * (bias - last_bias) + Velocity_KI * bias);

    last_bias = bias;

    // 输出限幅在 STM32 PWM 定时器满幅范围内 [-7200, +7200]
    if (pwm > 7200)  pwm = 7200;
    if (pwm < -7200) pwm = -7200;

    return pwm;
}

/* 清除控制器内部历史状态 (刹车/上锁切换时调用) */
static inline void Incremental_PI_Clear(void)
{
    Incremental_PI_A(0.0f, 0.0f);
    Incremental_PI_B(0.0f, 0.0f);
}

#endif /* __PID_H */
