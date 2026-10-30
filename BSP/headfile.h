#ifndef __HEADFILE_H_
#define __HEADFILE_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "tim.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "I2C.h"
#include "mpu6050.h"
#include "motor.h"
#include "pstwo.h"
#include "encoder.h"
#include "myadc.h"
#include "pid.h"
#include "vofa.h"
#include "car.h"
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart1_ON_Flag, Usart5_ON_Flag; 
extern uint8_t rx_buff;	


extern  float target_speed ;
extern  float target_speed_l ;
extern  float target_speed_r ;

extern  int encoder_l ;
extern  int encoder_r ;
extern	int current_pwm_l ;
extern  int current_pwm_r ;
extern  int target_servo ;
extern float Velocity_KP;
extern float Velocity_KI;



#endif
