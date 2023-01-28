#include "i2c_def.h"
#include "main.h"
#include "cmsis_os.h"
#include "GearLL_config.h"

extern void delay_us(uint16_t nus);
#ifdef USE_SOFT_IIC
	void MPU_I2C_Start(void)
	{
		MPU_SDA_1();
		MPU_SCL_1();
		delay_us(4);
		MPU_SDA_0();
		delay_us(4);
		MPU_SCL_0();
		delay_us(4);
	}
	void MPU_I2C_Stop(void)
	{
		MPU_SDA_0();
		MPU_SCL_1();
		delay_us(4);
		MPU_SDA_1();
	}
	
	void MPU_I2C_Ack(void)
	{
		MPU_SDA_0();	/* CPU????SDA = 0 */
		delay_us(4);
		MPU_SCL_1();	/* CPU????1????? */
		delay_us(4);
		MPU_SCL_0();
		delay_us(4);
		MPU_SDA_1();	/* CPU???SDA???? */
	}
	void MPU_I2C_NAck(void)
	{
		MPU_SDA_1();	/* CPU????SDA = 1 */
		delay_us(4);
		MPU_SCL_1();	/* CPU????1????? */
		delay_us(4);
		MPU_SCL_0();
		delay_us(4);	
	}
	
	//ack: 		0
	//not ack:1
	uint8_t MPU_I2C_WaitAck(void)
	{
		uint8_t result;
	
		MPU_SDA_1();	/* CPU???SDA???? */
		delay_us(4);
		MPU_SCL_1();	/* CPU????SCL = 1, ???????????ACK??? */
		delay_us(4);
		if (MPU_READSDA)	/* CPU???SDA?????? */
		{
			result = 1;
		}
		else
		{
			result = 0;
		}
		MPU_SCL_0();
		delay_us(4);
		return result;
	}
	
	void MPU_I2C_WriteByte(uint8_t txMsg)
	{
		uint8_t i;
		
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
			delay_us(4);
			MPU_SCL_1();
			delay_us(4);	
			MPU_SCL_0();
			if (i == 7)
			{
				MPU_SDA_1(); // ???????
			}
			txMsg <<= 1;	/* ???????bit */
			delay_us(4);
		}
	}
	
	
	//ack: 		 1
	//not ack: 0
	uint8_t MPU_I2C_ReadByte(uint8_t ack)
	{
		uint8_t i,rxMsg = 0;
	
		for (i = 0; i < 8; i++)
		{
			rxMsg <<= 1;
			MPU_SCL_1();
			delay_us(4);
			if (MPU_READSDA)
			{
				rxMsg++;
			}
			MPU_SCL_0();
			delay_us(4);
		}
		if(ack==0)
			MPU_I2C_NAck();
		else
			MPU_I2C_Ack();
		return rxMsg;
	}
#endif



