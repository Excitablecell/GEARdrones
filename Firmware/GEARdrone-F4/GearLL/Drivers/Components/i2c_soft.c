#include "i2c_def.h"
#include "main.h"
#include "cmsis_os.h"
#include "timer_driver.h"

extern TIM_HandleTypeDef htim4;
#define DLY_TIM_Handle (&htim4)

void delay_us(uint16_t nus)
{
	__HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
	__HAL_TIM_ENABLE(DLY_TIM_Handle);
	while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus)
	{
	}
	__HAL_TIM_DISABLE(DLY_TIM_Handle);
}

void TOF_I2C_Init(void)
{
	TOF_SDA_0();
	TOF_SCL_0();
	delay_us(50);
	TOF_SDA_1();
	TOF_SCL_1();
	delay_us(4);
	
}

void TOF_I2C_Start(void)
{
	TOF_SDA_1();
	TOF_SCL_1();
	delay_us(4);
	TOF_SDA_0();
	delay_us(4);
	TOF_SCL_0();
}
void TOF_I2C_Stop(void)
{
	TOF_SDA_0();
	TOF_SCL_0();
	delay_us(4);
	TOF_SCL_1();
	TOF_SDA_1();
	delay_us(4);
}

void TOF_I2C_Ack(void)
{
	TOF_SCL_0();
	TOF_SDA_0();
	delay_us(4);
	TOF_SCL_1();
	delay_us(4);
	TOF_SCL_0();
}
void TOF_I2C_NAck(void)
{
	TOF_SCL_0();
	TOF_SDA_1();
	delay_us(4);
	TOF_SCL_1();
	delay_us(4);
	TOF_SCL_0();
}

//ack: 		0
//not ack:1
uint8_t TOF_I2C_WaitAck(void)
{
	uint8_t ucErrTime=0;

	TOF_SDA_1();delay_us(4);	   
	TOF_SCL_1();delay_us(4);	 
	while(TOF_READSDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			TOF_I2C_Stop();
			return 1;
		}
	}
	TOF_SCL_0();//时钟输出0 	   
	return 0;  
}

void TOF_I2C_WriteByte(uint8_t txMsg)
{
	uint8_t i;

	TOF_SCL_0();//时钟输出0 	
	for (i = 0; i < 8; i++)
	{		
		if ((txMsg&0x80)>>7){TOF_SDA_1();}
		else{TOF_SDA_0();}
		txMsg <<= 1;	/* 左移一个bit */
		delay_us(4);
		TOF_SCL_1();
		delay_us(4);
		TOF_SCL_0();
		delay_us(4);
	}
}


//ack: 		 1
//not ack: 0
uint8_t TOF_I2C_ReadByte(unsigned char ack)
{
	uint8_t i;
	uint8_t rxMsg;

	/* 读到第1个bit为数据的bit7 */
	rxMsg = 0;
	TOF_SDA_1();
	delay_us(4);
	for (i = 0; i < 8; i++)
	{
		rxMsg <<= 1;
		TOF_SCL_0();
		delay_us(4);
		TOF_SCL_1();
		delay_us(4);
		if (TOF_READSDA){rxMsg |= 0x01;}
		delay_us(4);
	}
	if (!ack)
		TOF_I2C_NAck();//发送nACK
	else
		TOF_I2C_Ack(); //发送ACK   
	TOF_SCL_0();
	return rxMsg;
}

