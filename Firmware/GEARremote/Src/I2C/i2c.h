#ifndef   _I2C_H_
#define   _I2C_H_
#include  "main.h"
#include  "gpio.h"
#include  "tim.h"
//OLED_SCL C13
//OLED_SDA C14
//MPU_SCL  B6
//MPU_SDA  B7
//address of oled

//OLED part
#define IIC_SLAVE_ADDR 0x78

#define OLED_SCL_1()   {HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_SET);}
#define OLED_SCL_0()   {HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_RESET);}

#define OLED_SDA_1()   {HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_SET);}
#define OLED_SDA_0()   {HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_RESET);}

#define OLED_READSDA       HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port,OLED_SDA_Pin)

#define OLED_SDA_IN()  {GPIOC->CRH&=0XF0FFFFFF;GPIOC->CRH|=0X08000000;}
#define OLED_SDA_OUT() {GPIOC->CRH&=0XF0FFFFFF;GPIOC->CRH|=0X02000000;}

//void OLED_SDA_IN(void);
//void OLED_SDA_OUT(void);
void OLED_I2C_Init(void);

void OLED_I2C_Start(void);
void OLED_I2C_Stop(void);

void OLED_I2C_Ack(void);
void OLED_I2C_NAck(void);

uint8_t OLED_I2C_WaitAck(void);

void OLED_I2C_WriteByte(uint8_t txMsg);
uint8_t OLED_I2C_ReadByte(void);

void Write_IIC_Data(uint8_t IIC_Data);
void Write_IIC_Command(uint8_t IIC_Command);

//MPU6050 part
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR				0X68

#define MPU_SCL_1()   {HAL_GPIO_WritePin(MPU_SCL_GPIO_Port,MPU_SCL_Pin,GPIO_PIN_SET);}
#define MPU_SCL_0()   {HAL_GPIO_WritePin(MPU_SCL_GPIO_Port,MPU_SCL_Pin,GPIO_PIN_RESET);}

#define MPU_SDA_1()   {HAL_GPIO_WritePin(MPU_SDA_GPIO_Port,MPU_SDA_Pin,GPIO_PIN_SET);}
#define MPU_SDA_0()   {HAL_GPIO_WritePin(MPU_SDA_GPIO_Port,MPU_SDA_Pin,GPIO_PIN_RESET);}

#define MPU_READSDA       HAL_GPIO_ReadPin(MPU_SDA_GPIO_Port,MPU_SDA_Pin)

//#define MPU_SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0X80000000;}
//#define MPU_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0X20000000;}

void MPU_I2C_Init(void);

void MPU_I2C_Start(void);
void MPU_I2C_Stop(void);

void MPU_I2C_Ack(void);
void MPU_I2C_NAck(void);

uint8_t MPU_I2C_WaitAck(void);

void MPU_I2C_WriteByte(uint8_t txMsg);
uint8_t MPU_I2C_ReadByte(uint8_t ack);

void MPU_Write_Len(uint8_t reg,uint8_t size,uint8_t *buff);
void MPU_Read_Len(uint8_t reg,uint8_t size,uint8_t *buff);

void 	MPU_Write_Byte(uint8_t reg,uint8_t data);
uint8_t  MPU_Read_Byte(uint8_t reg);
#endif
