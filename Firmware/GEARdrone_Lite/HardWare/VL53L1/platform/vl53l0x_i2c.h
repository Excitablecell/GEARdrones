#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H

#include "cmsis_os.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK NANO STM32开发板
//VL53L0X IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2018/7/18
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2018-2028
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//IO方向设置
#define VL_SDA_IN()  {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=8<<12;}
#define VL_SDA_OUT() {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=3<<12;}


//IO操作函数	 
#define VL_IIC_SCL    PAout(2)      //SCL
#define VL_IIC_SDA    PAout(3) 		//SDA	 
#define VL_READ_SDA   PAin(3) 		//输入SDA 

//状态
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

//IIC操作函数
void VL53L0X_i2c_init(void);//初始化IIC的IO口

uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data);              //IIC写一个8位数据
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data);             //IIC写一个16位数据
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data);            //IIC写一个32位数据
uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count);//IIC连续写

uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata);             //IIC读一个8位数据
uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata);            //IIC读一个16位数据
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata);           //IIC读一个32位数据
uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count);  //IIC连续读

#endif 


