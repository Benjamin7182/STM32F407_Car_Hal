#include "encoder.h"

void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // 开启 TIM2 编码器接口
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 开启 TIM3 编码器接口
    
    // 如果配置了 TIM4 和 TIM5 也一并开启：
    // HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    // HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

/**
 * @brief  读取当前采样周期的脉冲增量，并清零计数器
 * @param  tim_x: 定时器编号 (2: 左轮, 3: 右轮)
 * @retval 增量脉冲值 (带符号：正数代表正转，负数代表反转)
 */
int Read_Encoder(uint8_t tim_x)
{
    int encoder_val = 0;
    switch (tim_x)
    {
        case 2:
            // 强转为 short 极为关键：0 减 1 变成 65535，强转 short 自动变为 -1
            encoder_val = (short) TIM2->CNT;
            TIM2->CNT=0; // 读完清零，为下一个周期做准备
            break;
            
        case 3:
            encoder_val = (short) TIM3->CNT;
            TIM3->CNT=0;
            break;
            
        default:
            encoder_val = 0;
            break;
    }
    return encoder_val;
}