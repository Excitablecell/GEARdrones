#ifndef   _I2C_H_
#define   _I2C_H_

#include  "main.h"
#include  "stm32f4xx_hal_gpio.h"
#include  "stm32f4xx_hal_tim.h"
#include "GearLL_config.h"

#ifdef __cplusplus    
extern "C" {         
#endif
//MPU6050 part
#define MPU_ADDR				0X68

#define MPU_SCL_1()   {HAL_GPIO_WritePin(MPU_SCL_GPIO_Port,MPU_SCL_Pin,GPIO_PIN_SET);}
#define MPU_SCL_0()   {HAL_GPIO_WritePin(MPU_SCL_GPIO_Port,MPU_SCL_Pin,GPIO_PIN_RESET);}

#define MPU_SDA_1()   {HAL_GPIO_WritePin(MPU_SDA_GPIO_Port,MPU_SDA_Pin,GPIO_PIN_SET);}
#define MPU_SDA_0()   {HAL_GPIO_WritePin(MPU_SDA_GPIO_Port,MPU_SDA_Pin,GPIO_PIN_RESET);}
#define MPU_READSDA       HAL_GPIO_ReadPin(MPU_SDA_GPIO_Port,MPU_SDA_Pin)

void MPU_I2C_Init(void);
void MPU_I2C_Start(void);
void MPU_I2C_Stop(void);
void MPU_I2C_Ack(void);
void MPU_I2C_NAck(void);

uint8_t MPU_I2C_WaitAck(void);
void MPU_I2C_WriteByte(uint8_t txMsg);
uint8_t MPU_I2C_ReadByte(uint8_t ack);
int MPU_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff);
int MPU_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff);
void 	MPU_Write_Byte(uint8_t reg,uint8_t data);
uint8_t  MPU_Read_Byte(uint8_t reg);

#define TOF_SCL_1()   {HAL_GPIO_WritePin(TOF_SCL_GPIO_Port,TOF_SCL_Pin,GPIO_PIN_SET);}
#define TOF_SCL_0()   {HAL_GPIO_WritePin(TOF_SCL_GPIO_Port,TOF_SCL_Pin,GPIO_PIN_RESET);}

#define TOF_SDA_1()   {HAL_GPIO_WritePin(TOF_SDA_GPIO_Port,TOF_SDA_Pin,GPIO_PIN_SET);}
#define TOF_SDA_0()   {HAL_GPIO_WritePin(TOF_SDA_GPIO_Port,TOF_SDA_Pin,GPIO_PIN_RESET);}
#define TOF_READSDA       HAL_GPIO_ReadPin(TOF_SDA_GPIO_Port,TOF_SDA_Pin)

void TOF_I2C_Init(void);
void TOF_I2C_Start(void);
void TOF_I2C_Stop(void);
void TOF_I2C_Ack(void);
void TOF_I2C_NAck(void);

uint8_t TOF_I2C_WaitAck(void);
void TOF_I2C_WriteByte(uint8_t txMsg);
uint8_t TOF_I2C_ReadByte(unsigned char ack);
int TOF_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff);
int TOF_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff);
void 	TOF_Write_Byte(uint8_t reg,uint8_t data);
uint8_t  TOF_Read_Byte(uint8_t reg);

#ifdef __cplusplus
}
#endif
#endif
