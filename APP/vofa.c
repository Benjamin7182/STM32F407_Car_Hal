#include "main.h"
#include "usart.h"
#include "vofa.h"

float a = 1.1f; // 测试调参变量

void Float_to_Byte(float f, unsigned char byte[])
{
	FloatLongType fl;
	fl.fdata = f;
	byte[0] = (unsigned char)fl.ldata;
	byte[1] = (unsigned char)(fl.ldata >> 8);
	byte[2] = (unsigned char)(fl.ldata >> 16);
	byte[3] = (unsigned char)(fl.ldata >> 24);
}

void Send_Data(UART_HandleTypeDef *huart, float f)
{
	unsigned char byte[4] = {0};
	Float_to_Byte(f, byte);
	HAL_UART_Transmit(huart, byte, 4, 10);
}

void Send_Tail(UART_HandleTypeDef *huart)
{
	unsigned char byte[4] = {0x00, 0x00, 0x80, 0x7f};
	HAL_UART_Transmit(huart, byte, 4, 10);
}

void vodka_JustFloat_send(UART_HandleTypeDef *huart)
{
	Send_Data(huart, (float)current_pwm_l);
	Send_Data(huart, (float)encoder_l);
	Send_Tail(huart);
}

/* 一次性打包发送 JustFloat 16 字节完整帧 (3 个 float 数据 + 00 00 80 7F 帧尾) */
void VOFA_Send_Speed(UART_HandleTypeDef *huart, float target, float left, float right)
{
	float buf[4];
	buf[0] = target;
	buf[1] = left;
	buf[2] = right;
	*((uint32_t *)&buf[3]) = 0x7F800000; // 对应字节 00 00 80 7F
	HAL_UART_Transmit(huart, (uint8_t *)buf, sizeof(buf), 10);
}

#define RXSTRSIZE 256
uint8_t rx_string[RXSTRSIZE];
uint8_t rx_cnt = 0;
uint8_t rx_buff;
uint8_t RxBuff;
unsigned char shujv[12];
int vofa_i = 0, vofa_I = 0;
float vofa_float;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t dat;
	if (huart->Instance == USART1)
	{
		ROS_Parse_Byte(ros_rx1_byte, 0);
		HAL_UART_Receive_IT(&huart1, &ros_rx1_byte, 1);
	}
	else if (huart->Instance == USART3)
	{
		ROS_Parse_Byte(ros_rx3_byte, 1);
		HAL_UART_Receive_IT(&huart3, &ros_rx3_byte, 1);
	}
	else if (huart->Instance == huart2.Instance)
	{
		dat = rx_buff;

		/* 防止超过 shujv[12] 边界导致覆盖后续 RAM 和主栈触发 HardFault */
		if (vofa_I < (int)sizeof(shujv) - 1)
		{
			shujv[vofa_I++] = dat;
		}
		else
		{
			vofa_I = 0;
		}

		if (dat == 0x0a)
		{
			for (vofa_i = 2, vofa_float = 0; vofa_i < vofa_I - 1; vofa_i++)
				vofa_float = 10 * vofa_float + (shujv[vofa_i] - '0');
			vofa_I = 0;

			if (shujv[1] == 0x3A)
			{
				switch (shujv[0])
				{
				case 'd':
					a = vofa_float / 10.0f;
					break;
				}
			}
		}

		HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buff, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_CLEAR_OREFLAG(huart);

	if (huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(&huart1, &ros_rx1_byte, 1);
	}
	else if (huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, &ros_rx3_byte, 1);
	}
	else if (huart->Instance == huart2.Instance)
	{
		vofa_I = 0;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buff, 1);
	}
}

