#ifndef   _I2C_H_
#define   _I2C_H_
#include  "main.h"
#include  "stm32f1xx_hal_gpio.h"
#include  "stm32f1xx_hal_tim.h"

//MPU6050 part
#define MPU_ADDR				0X68

void MPU_I2C_Init(void);
int MPU_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff);
int MPU_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff);
void 	MPU_Write_Byte(uint8_t reg,uint8_t data);
uint8_t  MPU_Read_Byte(uint8_t reg);

#define TOF_SCL_1()   {HAL_GPIO_WritePin(TOF_SCL_GPIO_Port,TOF_SCL_Pin,GPIO_PIN_SET);}
#define TOF_SCL_0()   {HAL_GPIO_WritePin(TOF_SCL_GPIO_Port,TOF_SCL_Pin,GPIO_PIN_RESET);}

#define TOF_SDA_1()   {HAL_GPIO_WritePin(TOF_SDA_GPIO_Port,TOF_SDA_Pin,GPIO_PIN_SET);}
#define TOF_SDA_0()   {HAL_GPIO_WritePin(TOF_SDA_GPIO_Port,TOF_SDA_Pin,GPIO_PIN_RESET);}

#define TOF_READSDA       HAL_GPIO_ReadPin(TOF_SDA_GPIO_Port,TOF_SDA_Pin)

#define SCL_H   HAL_GPIO_WritePin(TOF_SCL_GPIO_Port,TOF_SCL_Pin,GPIO_PIN_SET)
#define SCL_L   HAL_GPIO_WritePin(TOF_SCL_GPIO_Port,TOF_SCL_Pin,GPIO_PIN_RESET)

#define SDA_H   HAL_GPIO_WritePin(TOF_SDA_GPIO_Port,TOF_SDA_Pin,GPIO_PIN_SET)
#define SDA_L   HAL_GPIO_WritePin(TOF_SDA_GPIO_Port,TOF_SDA_Pin,GPIO_PIN_RESET)

#define SDA_read       HAL_GPIO_ReadPin(TOF_SDA_GPIO_Port,TOF_SDA_Pin)

extern void IIC_Init(void);                //初始化IIC的IO口

extern uint8_t IIC_ReadOneByte(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t* data);
extern unsigned char IICwriteByte(unsigned char dev, uint16_t reg, unsigned char data);
extern uint8_t IICwriteBytes(uint8_t dev, uint16_t reg, uint16_t length, uint8_t* data);
extern uint8_t IICwriteBit(uint8_t dev,uint16_t reg,uint8_t bitNum,uint8_t data);
extern uint8_t IICreadBytes(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t len,uint8_t *data);

void TOF_I2C_Init(void);

void TOF_I2C_Start(void);
void TOF_I2C_Stop(void);

void TOF_I2C_Ack(void);
void TOF_I2C_NAck(void);

uint8_t TOF_I2C_WaitAck(void);

void TOF_I2C_WriteByte(uint8_t txMsg);
void TOF_I2C_Init(void);

uint8_t TOF_I2C_ReadByte(unsigned char ack);

int TOF_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff);
int TOF_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff);

void 	TOF_Write_Byte(uint8_t reg,uint8_t data);
uint8_t  TOF_Read_Byte(uint8_t reg);

#endif
