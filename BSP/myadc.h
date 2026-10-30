#ifndef __MYADC_H_
#define __MYADC_H_

#include "main.h"

#define LOW_VOLTAGE_THRESHOLD  10.0f  // 3S 锂电池欠压保护门限 (10.0V)

extern float Battery_Voltage;
extern uint32_t Battery_ADC_Raw;

uint32_t Get_Battery_ADC_Raw(void);
float Get_Battery_Voltage(void);
uint8_t Check_Battery_Safe(void);

#endif
