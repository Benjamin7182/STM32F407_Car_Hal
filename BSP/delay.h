#ifndef __DELAY_H_
#define __DELAY_H_

#include "main.h"

void DWT_Delay_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif
