#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "headfile.h"

#define PWM_MAX  16799
#define MOTOR_EN HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) // 

#define PWMA1 	  TIM10->CCR1	 //PWMA
#define PWMA2 	  TIM11->CCR1	 //PWMA
#define PWMB1 	  TIM9->CCR1	 //PWMB
#define PWMB2 	  TIM9->CCR2	 //PWMB
#define PWMC1 	  TIM1->CCR2	 //PWMC
#define PWMC2 	  TIM1->CCR1	 //PWMC
#define PWMD1 	  TIM1->CCR4	 //PWMD
#define PWMD2 	  TIM1->CCR3	 //PWMD

#define Servo_PWM  TIM12->CCR2
#define SERVO_INIT 1500  //Servo zero point //???¨²¨¢?¦Ì?

void Motor_Init(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Motor_Stop(void);
void Set_Servo(int servo_val);

#endif
