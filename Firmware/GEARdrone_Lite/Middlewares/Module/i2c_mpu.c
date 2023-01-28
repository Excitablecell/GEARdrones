#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

void MPU_I2C_Init(void)
{
	if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDR, 1, 20000) != HAL_OK){};
}


void 	MPU_Write_Byte(uint8_t reg,uint8_t data)
{
	extern I2C_HandleTypeDef hi2c1;
  unsigned char W_Data=0;

  W_Data = data;
  HAL_I2C_Mem_Write(&hi2c1, ((MPU_ADDR<<1)|0X01), reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xff);
  HAL_Delay(1);
}
uint8_t  MPU_Read_Byte(uint8_t reg)
{
	uint8_t data;
	extern I2C_HandleTypeDef hi2c1;
  
  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xff);
  HAL_Delay(1);
	return data;
}
int MPU_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff)
{
	extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Write(&hi2c1, ((MPU_ADDR<<1)|0X01), reg, I2C_MEMADD_SIZE_8BIT, buff, size, 0xff);
	return 0;
}
int MPU_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff)
{
	extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Read(&hi2c1, ((MPU_ADDR<<1)|0X00), reg, I2C_MEMADD_SIZE_8BIT, buff, size, 0xff);
	return 0;
}

