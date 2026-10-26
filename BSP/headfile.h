#ifndef __HEADFILE_H_
#define __HEADFILE_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "I2C.h"
#include "mpu6050.h"
#include "motor.h"
#include "pstwo.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;


extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart1_ON_Flag, Usart5_ON_Flag; 
#endif
