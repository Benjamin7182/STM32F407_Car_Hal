#include "myadc.h"
#include "adc.h"

float Battery_Voltage = 0.0f;
uint32_t Battery_ADC_Raw = 0;

/**
 * @brief  获取 ADC 单次原始值 (12位, 0~4095)
 */
uint32_t Get_Battery_ADC_Raw(void)
{
    uint32_t adc_val = 0;
    uint32_t timeout = 50000;
    
    HAL_ADC_Start(&hadc1);
    while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) && --timeout);
    if (timeout > 0)
    {
        adc_val = HAL_ADC_GetValue(&hadc1);
    }
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_ADC_Stop(&hadc1);
    Battery_ADC_Raw = adc_val;
    return adc_val;
}

/**
 * @brief  单次读取转换为实际电池电压 (单位: V)
 */
float Get_Battery_Voltage_Raw(void)
{
    uint32_t adc_val = Get_Battery_ADC_Raw();
    // 计算公式: 电压 = 读数 / 4095 * 3.3V * 分压比(11.0) * 校准系数(0.99)
    float volt = (float)adc_val * 3.3f / 4095.0f * 11.0f * 0.99f;
    return volt;
}

/**
 * @brief  8次均值滤波更新全局 Battery_Voltage
 */
float Get_Battery_Voltage(void)
{
    float sum = 0.0f;
    for (int i = 0; i < 8; i++)
    {
        sum += Get_Battery_Voltage_Raw();
    }
    Battery_Voltage = sum / 8.0f;
    return Battery_Voltage;
}

/**
 * @brief  电压安全检测 (低于 10.0V 判定为欠压不安全)
 * @retval 1: 电压安全, 0: 严重欠压
 */
uint8_t Check_Battery_Safe(void)
{
    float v = Get_Battery_Voltage();
    if (v < LOW_VOLTAGE_THRESHOLD)
    {
        return 0; // 欠压停机
    }
    return 1;
}
