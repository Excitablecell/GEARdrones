#include "i2c_def.h"

#ifdef USE_SOFT_IIC

extern void delay_us(uint16_t nus);

void MPU_I2C_Init(void)
{
	MPU_SDA_0();
	MPU_SCL_0();
	delay_us(50);
	MPU_SDA_1();
	MPU_SCL_1();
	
	MPU_I2C_Stop();
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
int MPU_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff)
{
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|0X00);
	if(MPU_I2C_WaitAck())
	{
		MPU_I2C_Stop();
		return 1;
	}
	MPU_I2C_WriteByte(reg);
	MPU_I2C_WaitAck();
	for(uint8_t i=0;i<size;i++)
	{
		MPU_I2C_WriteByte(buff[i]);
		if(MPU_I2C_WaitAck())
		{
			MPU_I2C_Stop();
			return 1;
		}
	}
	MPU_I2C_Stop();
	return 0;
}
int MPU_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff)
{
	uint8_t i;
	MPU_I2C_Start();
	MPU_I2C_WriteByte((MPU_ADDR<<1)|0);
	if(MPU_I2C_WaitAck())
	{
		MPU_I2C_Stop();
	}
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
	return 0;
}
#endif

#ifndef USE_SOFT_IIC

extern I2C_HandleTypeDef hi2c1;

void MPU_I2C_Init(void){
	if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDR, 1, 20000) != HAL_OK);
}


void 	MPU_Write_Byte(uint8_t reg,uint8_t data){
	extern I2C_HandleTypeDef hi2c1;
  unsigned char W_Data=0;

  W_Data = data;
  HAL_I2C_Mem_Write(&hi2c1, ((MPU_ADDR<<1)|0X01), reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xff);
  HAL_Delay(1);
}
uint8_t  MPU_Read_Byte(uint8_t reg){
	uint8_t data;
	extern I2C_HandleTypeDef hi2c1;
  
  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xff);
  HAL_Delay(1);
	return data;
}
int MPU_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff){
	extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Write(&hi2c1, ((MPU_ADDR<<1)|0X01), reg, I2C_MEMADD_SIZE_8BIT, buff, size, 0xff);
//  HAL_Delay(1);
	return 0;
}
int MPU_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff){
	extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Read(&hi2c1, ((MPU_ADDR<<1)|0X00), reg, I2C_MEMADD_SIZE_8BIT, buff, size, 0xff);
//  HAL_Delay(1);
	return 0;
}
#endif



