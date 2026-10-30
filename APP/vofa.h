#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include "usart.h"
#include <string.h>
#include "headfile.h"
typedef union
{
	float fdata;
	unsigned long ldata;
} FloatLongType;

extern float a;

void Float_to_Byte(float f, unsigned char byte[]);
void Send_Data(UART_HandleTypeDef *huart, float f);
void Send_Tail(UART_HandleTypeDef *huart);
void vodka_JustFloat_send(UART_HandleTypeDef *huart);


void VOFA_Send_Speed(UART_HandleTypeDef *huart, float target, float left, float right);

#endif
