/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    HMC5883L.cpp
  * @author  EXcai
  * @brief   传感器HMC5883L驱动
  *
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *

  *******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus    
extern "C" {         
#endif
	
#include "i2c.h"

#ifdef __cplusplus
}
#endif
#include "HMC5883L.h"

uint8_t QMC_Data_Status(){
	
	uint8_t Sta;
	
	TOF_I2C_Start();
  TOF_I2C_WriteByte(0x1a); 
  TOF_I2C_WaitAck(); 
  TOF_I2C_WriteByte(0x06);
  TOF_I2C_WaitAck(); 
  TOF_I2C_Stop();

  TOF_I2C_Start();          
  TOF_I2C_WriteByte(0x1b); 
  TOF_I2C_WaitAck();
	Sta = TOF_I2C_ReadByte(1); 
	TOF_I2C_Stop();
	
	return Sta;
}

uint8_t HMC5883L_Init(void)
{
	uint8_t Sta;
	/* 软复位*/
	TOF_I2C_Start();
  TOF_I2C_WriteByte(0x1a); //写指令
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x0A);
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x80); 
  TOF_I2C_WaitAck();
  TOF_I2C_Stop();
	TOF_I2C_Start();
	
  TOF_I2C_WriteByte(0x1a); //写指令
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x0B);
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x01); 
  TOF_I2C_WaitAck();
  TOF_I2C_Stop();

	TOF_I2C_Start();
  TOF_I2C_WriteByte(0x1a); //写指令
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x20);
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x40); 
  TOF_I2C_WaitAck();
  TOF_I2C_Stop();
	
	TOF_I2C_Start();
  TOF_I2C_WriteByte(0x1a); //写指令
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x21);
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x01); 
  TOF_I2C_WaitAck();
  TOF_I2C_Stop();
	
  TOF_I2C_Start();
  TOF_I2C_WriteByte(0x1a); //写指令
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x09);
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x1D); //测量范围8G,连续测量模式,采样速率512hz,输出速率100hz
  TOF_I2C_WaitAck();
  TOF_I2C_Stop();
	
	TOF_I2C_Start();
  TOF_I2C_WriteByte(0x1a); //写指令
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x0A);
  TOF_I2C_WaitAck();
  TOF_I2C_WriteByte(0x41); //不复位，自动滚动读取指针，关闭中断模式
  TOF_I2C_WaitAck();
  TOF_I2C_Stop();
	
	TOF_I2C_Start();
  TOF_I2C_WriteByte(0x1a); 
  TOF_I2C_WaitAck(); 
  TOF_I2C_WriteByte(0x0D);
  TOF_I2C_WaitAck(); 
  TOF_I2C_Stop();

  TOF_I2C_Start();          
  TOF_I2C_WriteByte(0x1b); 
  TOF_I2C_WaitAck();
	Sta = TOF_I2C_ReadByte(1); 
//	TOF_I2C_Stop();
	return Sta;
}
void HMC5883L_ReadAngle(int16_t *x,int16_t *y,int16_t *z){

	uint8_t XYZ_Data[6]={0};
 
	TOF_I2C_Start();
	TOF_I2C_WriteByte(0x1A); 
	TOF_I2C_WaitAck(); 
	TOF_I2C_WriteByte(0x00);  //X轴数据地址
	TOF_I2C_WaitAck(); 
	TOF_I2C_Stop();

	TOF_I2C_Start();          
	TOF_I2C_WriteByte(0x1B); 
	TOF_I2C_WaitAck();
	
	for(uint8_t i=0;i<5;i++)     
	{
	XYZ_Data[i]=TOF_I2C_ReadByte(1);
	}

	XYZ_Data[5] =TOF_I2C_ReadByte(0);  
	TOF_I2C_Stop();
	
	*x = (int16_t)(XYZ_Data[1]<<8)|XYZ_Data[0];
	*y = (int16_t)(XYZ_Data[3]<<8)|XYZ_Data[2];
	*z = (int16_t)(XYZ_Data[5]<<8)|XYZ_Data[4];
}
