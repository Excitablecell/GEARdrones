#include "i2c.h"
//OLED_SCL C13
//OLED_SDA C14
//MPU_SCL  B6
//MPU_SDA  B7
//GPIO_InitTypeDef   GPIO_InitStruct;

//void OLED_SDA_OUT(void)
//{
//	
//GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//GPIO_InitStruct.Pin=OLED_SDA_Pin;
//HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//}

//void OLED_SDA_IN(void)
//{
//GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//GPIO_InitStruct.Pin=OLED_SDA_Pin;
//HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//}
void OLED_I2C_Init(void)
{
	OLED_SDA_OUT();
	OLED_SCL_1();
  OLED_SDA_1();
}

void OLED_I2C_Start(void)
{
	OLED_SDA_OUT();
	OLED_SCL_1();
	OLED_SDA_1();
	Delay_US(4);
	OLED_SDA_0();
	OLED_SCL_0();
}
		
void OLED_I2C_Stop(void)
{
	OLED_SDA_OUT();
	OLED_SCL_0();
	OLED_SDA_0();
	Delay_US(4);
	OLED_SCL_1();
	Delay_US(4);
	OLED_SDA_1();
}

void OLED_I2C_Ack(void)
{
	OLED_SDA_OUT();
	OLED_SCL_0();
	OLED_SDA_0();
	Delay_US(2);
	OLED_SCL_1();
	Delay_US(4);
	OLED_SCL_0();
}

void OLED_I2C_NAck(void)
{
	OLED_SDA_OUT();
	OLED_SCL_0();
	OLED_SDA_1();
	Delay_US(2);
	OLED_SCL_1();
	Delay_US(4);
	OLED_SCL_0();
}
//ack: 		1
//not ack:0
uint8_t OLED_I2C_WaitAck(void)
{
	uint8_t errTime=0;
	OLED_SDA_IN();
	OLED_SDA_1();//release the ctrl right
	Delay_US(1);
	OLED_SCL_1();
	Delay_US(1);
  while(OLED_READSDA)
	{
	errTime++;
		if(errTime>250)
		{
		OLED_I2C_Stop();
		return 0;
		}
	}
	OLED_SCL_0();
	return 1;
}
//@param:txMsg
void OLED_I2C_WriteByte(uint8_t txMsg)
{
	uint8_t i=0;
	OLED_SDA_OUT();
	OLED_SCL_0();
	for(;i<8;i++)
	{
		if((txMsg&0x80))
		{
		OLED_SDA_1();
		}
		else
		{
		OLED_SDA_0();
		}
		txMsg<<=1;
		Delay_US(2);
		OLED_SCL_1();
		Delay_US(2);
		OLED_SCL_0();
		Delay_US(2);
	}
}

uint8_t OLED_I2C_ReadByte(void)
{
	uint8_t rxMsg=0;
	OLED_SDA_IN();
	for(int i=0;i<8;i++)
	{
		OLED_SCL_0();
		Delay_US(2);
		OLED_SCL_1();
		rxMsg<<=1;	
		rxMsg|=OLED_READSDA;
//		rxMsg|=OLED_READSDA;
//		rxMsg<<=1;	
	}
	return rxMsg;
}

void Write_IIC_Command(uint8_t IIC_Command)
{
	OLED_I2C_Start();
	OLED_I2C_WriteByte(IIC_SLAVE_ADDR);            //Slave address,SA0=0
	OLED_I2C_WaitAck();	
	OLED_I2C_WriteByte(0x00);			//write command
	OLED_I2C_WaitAck();	
	OLED_I2C_WriteByte(IIC_Command); 
	OLED_I2C_WaitAck();	
	OLED_I2C_Stop();
}

void Write_IIC_Data(uint8_t IIC_Data)
{
	OLED_I2C_Start();
	OLED_I2C_WriteByte(IIC_SLAVE_ADDR);			//D/C#=0; R/W#=0
	OLED_I2C_WaitAck();	
	OLED_I2C_WriteByte(0x40);			//write data
	OLED_I2C_WaitAck();	
	OLED_I2C_WriteByte(IIC_Data);
	OLED_I2C_WaitAck();	
	OLED_I2C_Stop();
}

static void i2c_Delay(void)
{
	uint8_t i;
	for (i = 0; i < 10; i++);
}

void MPU_I2C_Init(void)
{
//	MPU_SDA_OUT();
	MPU_SDA_1();
	MPU_SCL_1();

	MPU_I2C_Stop();
}

void MPU_I2C_Start(void)
{
//	MPU_SDA_OUT();
//	MPU_SCL_1();
//	MPU_SDA_1();
	MPU_SDA_1();
	MPU_SCL_1();
	i2c_Delay();
	MPU_SDA_0();
	i2c_Delay();
	MPU_SCL_0();
	i2c_Delay();
}
void MPU_I2C_Stop(void)
{
	MPU_SDA_0();
	MPU_SCL_1();
	i2c_Delay();
	MPU_SDA_1();
}

void MPU_I2C_Ack(void)
{
	MPU_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	MPU_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	MPU_SCL_0();
	i2c_Delay();
	MPU_SDA_1();	/* CPU释放SDA总线 */
}
void MPU_I2C_NAck(void)
{
	MPU_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	MPU_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	MPU_SCL_0();
	i2c_Delay();	
}

//ack: 		1
//not ack:0
uint8_t MPU_I2C_WaitAck(void)
{
	uint8_t result;

	MPU_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	MPU_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (MPU_READSDA)	/* CPU读取SDA口线状态 */
	{
		result = 1;
	}
	else
	{
		result = 0;
	}
	MPU_SCL_0();
	i2c_Delay();
	return result;
}

void MPU_I2C_WriteByte(uint8_t txMsg)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (txMsg & 0x80)
		{
			MPU_SDA_1();
		}
		else
		{
			MPU_SDA_0();
		}
		i2c_Delay();
		MPU_SCL_1();
		i2c_Delay();	
		MPU_SCL_0();
		if (i == 7)
		{
			 MPU_SDA_1(); // 释放总线
		}
		txMsg <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}


//ack: 		 1
//not ack: 0
uint8_t MPU_I2C_ReadByte(uint8_t ack)
{
	uint8_t i;
	uint8_t rxMsg;

	/* 读到第1个bit为数据的bit7 */
	rxMsg = 0;
	for (i = 0; i < 8; i++)
	{
		rxMsg <<= 1;
		MPU_SCL_1();
		i2c_Delay();
		if (MPU_READSDA)
		{
			rxMsg++;
		}
		MPU_SCL_0();
		i2c_Delay();
	}
	if(ack==0)
		MPU_I2C_NAck();
	else
		MPU_I2C_Ack();
	return rxMsg;
}


void 	MPU_Write_Byte(uint8_t reg,uint8_t data)
{
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|0X00);
	MPU_I2C_WaitAck();
	MPU_I2C_WriteByte(reg);
	MPU_I2C_WaitAck();
	MPU_I2C_WriteByte(data);
	MPU_I2C_WaitAck();
	MPU_I2C_Stop();
}
uint8_t  MPU_Read_Byte(uint8_t reg)
{
	uint8_t data;
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|0X00);
	MPU_I2C_WaitAck();
	MPU_I2C_WriteByte(reg);
	MPU_I2C_WaitAck();
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|0X01);
	MPU_I2C_WaitAck();
	data= MPU_I2C_ReadByte(0);
	MPU_I2C_Stop();
	return data;
}

void MPU_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff)
{
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|0X00);
	MPU_I2C_WaitAck();
	MPU_I2C_WriteByte(reg);
	MPU_I2C_WaitAck();
	for(uint8_t i=0;i<size;i++)
	{
		MPU_I2C_WriteByte(buff[i]);
		MPU_I2C_WaitAck();
	}
	MPU_I2C_Stop();
}
void MPU_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff)
{
	uint8_t i;
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|0);
	MPU_I2C_WaitAck();
	MPU_I2C_WriteByte(reg);
	MPU_I2C_WaitAck();
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|1);
	MPU_I2C_WaitAck();
	for(i=0;i<(size-1);i++)
	{
		*buff=MPU_I2C_ReadByte(1);
//		MPU_I2C_WaitAck();
		buff++;
	}
	*buff=MPU_I2C_ReadByte(0);
	MPU_I2C_Stop();
}

