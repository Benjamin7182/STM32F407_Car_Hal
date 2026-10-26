#ifndef _I2C_H_
#define _I2C_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "math.h"
#include "string.h"
#include "delay.h"
#define u8    uint8_t
#define u16   uint16_t
#define u32   uint32_t

#define SDA_IN()  
#define SDA_OUT() 

//IO²Ù×÷º¯Êý	 
#define IIC_SCL(n)   HAL_GPIO_WritePin(MPU6050_SCL_GPIO_Port, MPU6050_SCL_Pin, (n) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define IIC_SDA(n)   HAL_GPIO_WritePin(MPU6050_SDA_GPIO_Port, MPU6050_SDA_Pin, (n) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define READ_SDA   HAL_GPIO_ReadPin(MPU6050_SDA_GPIO_Port, MPU6050_SDA_Pin)


#ifndef I2C_Direction_Transmitter
	#define  I2C_Direction_Transmitter      ((uint8_t)0x00)
#endif

#ifndef I2C_Direction_Receiver
	#define  I2C_Direction_Receiver         ((uint8_t)0x01)
#endif

enum
{
	I2C_ACK,
	I2C_NACK
};

void I2C_SDAMode(uint8_t Mode);
void I2C_Start(void);
void I2C_Stop(void);
bool I2C_WaiteForAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data);
bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data);
void I2C_WriteByte(uint8_t Data);
uint8_t I2C_ReadByte(uint8_t Ack);
u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);

void I2C_GPIOInit(void);

#endif

