#include "myadc.h"
#include "adc.h"

float Battery_Voltage = 0.0f;
uint32_t Battery_ADC_Raw = 0;

/**
 * @brief  鑾峰彇 ADC 鍗曟鍘熷鍊?(12浣? 0~4095)
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
 * @brief  鍗曟璇诲彇杞崲涓哄疄闄呯數姹犵數鍘?(鍗曚綅: V)
 */
float Get_Battery_Voltage_Raw(void)
{
    uint32_t adc_val = Get_Battery_ADC_Raw();
    // 璁＄畻鍏紡: 鐢靛帇 = 璇绘暟 / 4095 * 3.3V * 鍒嗗帇姣?11.0) * 鏍″噯绯绘暟(0.99)
    float volt = (float)adc_val * 3.3f / 4095.0f * 11.0f * 0.99f;
    return volt;
}

/**
 * @brief  8娆″潎鍊兼护娉㈡洿鏂板叏灞€ Battery_Voltage
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
 * @brief  鐢靛帇瀹夊叏妫€娴?(浣庝簬 10.0V 鍒ゅ畾涓烘瑺鍘嬩笉瀹夊叏)
 * @retval 1: 鐢靛帇瀹夊叏, 0: 涓ラ噸娆犲帇
 */
uint8_t Check_Battery_Safe(void)
{
    float v = Get_Battery_Voltage();
    if (v < LOW_VOLTAGE_THRESHOLD)
    {
        return 0; // 娆犲帇鍋滄満
    }
    return 1;
}
