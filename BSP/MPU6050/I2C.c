#include "I2C.h"

/**************************************************************************
Function: IIC pin initialization
Input   : none
Output  : none
功能：IIC引脚初始化（PB10-SCL, PB11-SDA）
**************************************************************************/
void I2C_GPIOInit(void)
{
    /* GPIO 已在 CubeMX 的 MX_GPIO_Init() 中完成开漏上拉初始化 */
    /* 此处只需确保 I2C 总线处于空闲状态（高电平） */
    IIC_SCL(1);
    IIC_SDA(1);
}

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : none
功能：模拟IIC起始信号
**************************************************************************/
void I2C_Start(void)
{
    SDA_OUT();     // sda输出
    IIC_SDA(1);
    if(!READ_SDA) return;	
    IIC_SCL(1);
    delay_us(1);
    IIC_SDA(0);    // START: when CLK is high, DATA change form high to low 
    if(READ_SDA) return;
    delay_us(1);
    IIC_SCL(0);    // 钳住I2C总线，准备发送或接收数据 
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
功能：模拟IIC结束信号
**************************************************************************/
void I2C_Stop(void)
{
    SDA_OUT();     // sda输出
    IIC_SCL(0);
    IIC_SDA(0);    // STOP: when CLK is high DATA change form low to high
    delay_us(1);
    IIC_SCL(1); 
    IIC_SDA(1);    // 发送I2C总线结束信号
    delay_us(1);	
}

/**************************************************************************
Function: IIC Wait for ACK
Input   : none
Output  : 1: ACK received, 0: NACK / Timeout
功能：等待应答信号
**************************************************************************/
bool I2C_WaiteForAck(void)
{
    u8 ucErrTime = 0;
    SDA_IN();      // SDA设置为输入  
    IIC_SDA(1);
    delay_us(1);	   
    IIC_SCL(1);
    delay_us(1);	 
    while(READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime > 50)
        {
            I2C_Stop();
            return 0;
        }
        delay_us(1);
    }
    IIC_SCL(0);    // 时钟输出0 	   
    return 1;
}

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
功能：IIC产生ACK应答
**************************************************************************/
void I2C_Ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);
    delay_us(1);
    IIC_SCL(1);
    delay_us(1);
    IIC_SCL(0);
}

/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
功能：IIC不产生ACK应答
**************************************************************************/ 
void I2C_NAck(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);
    delay_us(1);
    IIC_SCL(1);
    delay_us(1);
    IIC_SCL(0);
}

bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{
    uint8_t Dat, Mask;
    
    Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**************************************************************************
Function: IIC sends a byte
Input   : Data: 待发送数据
Output  : none
功能：IIC发送一个字节
**************************************************************************/
void I2C_WriteByte(uint8_t Data)
{
    u8 t;   
    SDA_OUT(); 	    
    IIC_SCL(0);    // 拉低时钟开始数据传输
    for(t=0; t<8; t++)
    {              
        IIC_SDA((Data & 0x80) >> 7);
        Data <<= 1; 	  
        delay_us(1);   
        IIC_SCL(1);
        delay_us(1); 
        IIC_SCL(0);	
        delay_us(1);
    }	 
}

/**************************************************************************
Function: IIC writes one byte to register
**************************************************************************/
u8 I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaiteForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaiteForAck();
    I2C_WriteByte(Data);
    I2C_WaiteForAck();
    I2C_Stop();
    return 1;
}

/**************************************************************************
Function: IIC writes multiple bytes to register
**************************************************************************/
bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
    uint8_t i;

    if(0 == Num || NULL == pBuff)
    {
        return false;
    }
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaiteForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaiteForAck();
    
    for(i = 0; i < Num; i++)
    {
        I2C_WriteByte(*(pBuff + i));
        I2C_WaiteForAck();
    }
    I2C_Stop();

    return true;
}

/**************************************************************************
Function: IIC reads a byte
Input   : Ack: 1发送ACK, 0发送NACK
Output  : 读取到的字节数据
功能：IIC读取一个字节
**************************************************************************/
uint8_t I2C_ReadByte(uint8_t Ack)
{
    uint8_t i, RecDat = 0;

    SDA_IN();
    for(i = 0; i < 8; i++)
    {
        IIC_SCL(0);
        delay_us(1);
        IIC_SCL(1);
        RecDat <<= 1;
        if(READ_SDA)
            RecDat |= 0x01;
        else
            RecDat &= ~0x01;
        delay_us(1);
    }
    if(I2C_ACK == Ack)
        I2C_Ack();
    else
        I2C_NAck();

    return RecDat;
}

/**************************************************************************
Function: IIC reads one byte from register
**************************************************************************/
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t TempVal = 0;
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaiteForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaiteForAck();
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
    I2C_WaiteForAck();
    TempVal = I2C_ReadByte(I2C_NACK);
    I2C_Stop();
    
    return TempVal;
}

/**************************************************************************
Function: IIC reads multiple bytes from register
**************************************************************************/
bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
    uint8_t i;

    if(0 == Num || NULL == pBuff)
    {
        return false;
    }
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaiteForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaiteForAck();
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
    I2C_WaiteForAck();

    for(i = 0; i < Num; i++)
    {
        if((Num - 1) == i)
        {
            *(pBuff + i) = I2C_ReadByte(I2C_NACK);
        }
        else
        {
            *(pBuff + i) = I2C_ReadByte(I2C_ACK);
        }
    }

    I2C_Stop();
    
    return true;
}
