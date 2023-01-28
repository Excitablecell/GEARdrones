/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    SPL06_001.cpp
  * @author  EXcai
  * @brief   传感器SPL06气压计驱动
  *
  ==============================================================================
													How to use this library 
  ==============================================================================
    @note
			- 传感器SPL06气压计驱动

  	@warning 
			- Standard C++11 required! 
  
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
	
#include "i2c.h"

#ifdef __cplusplus
}
#endif

#include "SPL06_001.h"

uint8_t SPL06_Write_Byte(uint8_t addr,uint8_t data)
{
  TOF_I2C_Start();
	TOF_I2C_WriteByte(0xEE);
	if(TOF_I2C_WaitAck()){TOF_I2C_Stop(); return 0;} 
	TOF_I2C_WriteByte(addr);
	if(TOF_I2C_WaitAck()){TOF_I2C_Stop(); return 0;} 
	TOF_I2C_WriteByte(data);
	if(TOF_I2C_WaitAck()){TOF_I2C_Stop(); return 0;} 
	TOF_I2C_Stop();
	return 1;
}

uint8_t SPL06_Read_Byte(uint8_t addr)
{
	uint8_t SPL06_Data;
	
	TOF_I2C_Start();
	TOF_I2C_WriteByte(0xEE);
	if(TOF_I2C_WaitAck()){TOF_I2C_Stop(); return 0;} 
	TOF_I2C_WriteByte(addr);
	if(TOF_I2C_WaitAck()){TOF_I2C_Stop(); return 0;} 
 
	TOF_I2C_Start();// start again
	TOF_I2C_WriteByte(0xEF);
	if(TOF_I2C_WaitAck()){TOF_I2C_Stop(); return 0;} 
	SPL06_Data = TOF_I2C_ReadByte(0);
	TOF_I2C_Stop();
	return SPL06_Data;
}

uint8_t SPL06_Init(void)
{
	uint8_t SPL06_ID;
	TOF_I2C_Init();
	SPL06_Write_Byte(RESET_Addr,0x89);//Reset
	HAL_Delay(50);
	SPL06_ID = SPL06_Read_Byte(ID_Addr);//Read the SPL06's ID
	SPL06_Write_Byte(MEAS_CFG_Addr,0x07);//Set Working mode and state of sensor
	SPL06_Write_Byte(PRS_CFG_Addr,0x27);//Set the PM-RATE and PM-PRC
	SPL06_Write_Byte(TMP_CFG_Addr,0xA0);//Set the TMPI-RATE and TMP-PRC
	SPL06_Write_Byte(CFG_REG_Addr,0x04);//Configuration of abort, measurement data shift and FIFO enable
	return SPL06_ID;
}

float Temperature_conversion(uint32_t Temp_Data,float k)
{
	float Temperature;
	int Temp;
	if(Temp_Data&0x800000)
	{
		Temp = Temp_Data-Total_Number_24;
	}
	else
	{
		Temp = Temp_Data;
	}
	Temperature = Temp/k;
	return Temperature;
}
 
float Pressure_conversion(uint32_t Pressure_Data,float k)
{
	int Press;
	if(Pressure_Data&0x800000)
	{
		Press = Pressure_Data-Total_Number_24;
	}
	else
	{
		Press = Pressure_Data;
	}
	return (float)Press/k;
}
 
float Scale_factor(uint8_t Config_k)
{
	float k;
	switch(Config_k)
	{
		case 0: k = k_SPS1;break;
		case 1: k = k_SPS2;break;
    case 2: k = k_SPS4;break;
		case 3: k = k_SPS8;break;
		case 4: k = k_SPS16;break;
		case 5: k = k_SPS32;break;
    case 6: k = k_SPS64;break;
    case 7:	k = k_SPS128;break;	 	
	}
	return k;
}
 
void Parameter_Reading(int *Pressure_Para,int *Temperature_Para)
{
	uint8_t Temp_Config[3],Press_Config[15];	 
	
	//Temperature
	Temp_Config[0] = SPL06_Read_Byte(Temp_c0_Addr);
	Temp_Config[1] = SPL06_Read_Byte(Temp_c1_Addr);
	Temp_Config[2] = SPL06_Read_Byte(Temp_c2_Addr);
	Temperature_Para[0] = (Temp_Config[0]<<4)+((Temp_Config[1]&0xF0)>>4);
	if(Temperature_Para[0]&0x0800) Temperature_Para[0] = Temperature_Para[0]-Total_Number_12;
	Temperature_Para[1] = ((Temp_Config[1]&0x0F)<<8)+Temp_Config[2];
	if(Temperature_Para[1]&0x0800) Temperature_Para[1] = Temperature_Para[1]-Total_Number_12;
	//Pressure
	Press_Config[0] = SPL06_Read_Byte(Press_c0_Addr);
	Press_Config[1] = SPL06_Read_Byte(Press_c1_Addr);
	Press_Config[2] = SPL06_Read_Byte(Press_c2_Addr);
	Press_Config[3] = SPL06_Read_Byte(Press_c3_Addr);
	Press_Config[4] = SPL06_Read_Byte(Press_c4_Addr);
	Press_Config[5] = SPL06_Read_Byte(Press_c5_Addr);
	Press_Config[6] = SPL06_Read_Byte(Press_c6_Addr);
	Press_Config[7] = SPL06_Read_Byte(Press_c7_Addr);
	Press_Config[8] = SPL06_Read_Byte(Press_c8_Addr);
	Press_Config[9] = SPL06_Read_Byte(Press_c9_Addr);
	Press_Config[10] = SPL06_Read_Byte(Press_c10_Addr);
	Press_Config[11] = SPL06_Read_Byte(Press_c11_Addr);
	Press_Config[12] = SPL06_Read_Byte(Press_c12_Addr);
	Press_Config[13] = SPL06_Read_Byte(Press_c13_Addr);
	Press_Config[14] = SPL06_Read_Byte(Press_c14_Addr);
	Pressure_Para[0] = (Press_Config[0]<<12)+(Press_Config[1]<<4)+((Press_Config[2]&0xF0)>>4);//c00
	if(Pressure_Para[0]&0x80000) Pressure_Para[0] = Pressure_Para[0] - Total_Number_20;//c00
	Pressure_Para[1] = ((Press_Config[2]&0x0F)<<16)+ (Press_Config[3]<<8)+ Press_Config[4];//c10
	if(Pressure_Para[1]&0x80000) Pressure_Para[1] = Pressure_Para[1] - Total_Number_20;//c10
	Pressure_Para[2] = (Press_Config[5]<<8)+Press_Config[6];//c01
	if(Pressure_Para[2]&0x8000) Pressure_Para[2] = Pressure_Para[2] - Total_Number_16;//c01
	Pressure_Para[3] = (Press_Config[7]<<8)+Press_Config[8];//c11
	if(Pressure_Para[3]&0x8000) Pressure_Para[3] = Pressure_Para[3] - Total_Number_16;//c11
	Pressure_Para[4] = (Press_Config[9]<<8)+Press_Config[10];//c20
	if(Pressure_Para[4]&0x8000) Pressure_Para[4] = Pressure_Para[4] - Total_Number_16;//c20
	Pressure_Para[5] = (Press_Config[11]<<8)+Press_Config[12];//c21
	if(Pressure_Para[5]&0x8000) Pressure_Para[5] = Pressure_Para[5] - Total_Number_16;//c21
	Pressure_Para[6] = (Press_Config[13]<<8)+Press_Config[14];//c30
	if(Pressure_Para[6]&0x8000) Pressure_Para[6] = Pressure_Para[6] - Total_Number_16;//c30
}
float Correcting_Pressure(int *Pressure_Para,float Pressure,float Temperature)
{
	return Pressure_Para[0]+ Pressure*(Pressure_Para[1]+Pressure*(Pressure_Para[4]+Pressure*Pressure_Para[6]))+Temperature*Pressure_Para[2]+Temperature*Pressure*(Pressure_Para[3]+Pressure*Pressure_Para[5]);
}
 
float Correcting_Temperature(int *Temperature_Para,float Temperature)
{	
	return Temperature_Para[0]*0.5+Temperature_Para[1]*Temperature;
}


