/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    SPL06_001.h
  * @author  EXcai
  * @brief   传感器SPL06气压计驱动
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
#ifndef __SPL06_001_H_
#define __SPL06_001_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "math.h"
#include "Butterworth.h"

#define SPL06_Write 0XEE
#define SPL06_Read 0xEF
 
#define k_SPS1 524288.0
#define k_SPS2 1572864.0
#define k_SPS4 3670016.0
#define k_SPS8 7864320.0
#define k_SPS16 253952.0
#define k_SPS32 516096.0
#define k_SPS64 1040384.0
#define k_SPS128 2088960.0
 
#define PSR_B2_Addr 0x00
#define PSR_B1_Addr 0x01
#define PSR_B0_Addr 0x02
#define TMP_B2_Addr 0x03
#define TMP_B1_Addr 0x04
#define TMP_B0_Addr 0x05
#define PRS_CFG_Addr 0x06
#define TMP_CFG_Addr 0x07
#define MEAS_CFG_Addr 0x08
#define CFG_REG_Addr 0x09
#define RESET_Addr 0x0C
#define ID_Addr 0x0D
 
#define Temp_c0_Addr 0x10
#define Temp_c1_Addr  0x11
#define Temp_c2_Addr  0x12
 
#define Press_c0_Addr  0x13
#define Press_c1_Addr  0x14
#define Press_c2_Addr  0x15
#define Press_c3_Addr  0x16
#define Press_c4_Addr  0x17
#define Press_c5_Addr  0x18
#define Press_c6_Addr  0x19
#define Press_c7_Addr  0x1A
#define Press_c8_Addr  0x1B
#define Press_c9_Addr  0x1C
#define Press_c10_Addr  0x1D
#define Press_c11_Addr  0x1E
#define Press_c12_Addr  0x1F
#define Press_c13_Addr  0x20
#define Press_c14_Addr  0x21
 
#define Total_Number_24 16777216.0
#define Total_Number_20 1048576.0
#define Total_Number_16 65536.0
#define Total_Number_12 4096.0

/* Variables -------------------------------------------------------------------*/

uint8_t SPL06_Init(void);
uint8_t SPL06_Read_Byte(uint8_t addr);
uint8_t SPL06_Write_Byte(uint8_t addr,uint8_t data);
void Parameter_Reading(int *Pressure_Para,int *Temperature_Para);
float Temperature_conversion(uint32_t Temp_Data,float k);
float Pressure_conversion(uint32_t Pressure_Data,float k);
float Scale_factor(uint8_t Config_k);
float Correcting_Pressure(int *Pressure_Para,float Pressure,float Temperature);
float Correcting_Temperature(int *Temperature_Para,float Temperature);

/* 二阶巴特沃斯低通滤波器系数*/ 
static float IIRCoeffs32LP_barometer[5] = {1.0f, 2.0f, 1.0f, 1.143f,-0.413f};
static ButterworthFilter BWF_barometer(IIRCoeffs32LP_barometer,0.0674f,1);

/** 
* @brief Class for SPL06.
*/
class SPL06{
public:
	SPL06(){};
	~SPL06(){};
	uint8_t SPL06_ID,Sensor_state;
	uint8_t Init(){
		
		uint8_t Sta;
		
		Sta = SPL06_Init();
		Parameter_Reading(Pressure_Para,Temperature_Para);
		Config_Press = SPL06_Read_Byte(PRS_CFG_Addr);
		k_Press = Scale_factor((Config_Press)&0x0F);
		Config_Temp = SPL06_Read_Byte(TMP_CFG_Addr);
		k_Temp = Scale_factor((Config_Temp)&0x07);
		Config = SPL06_Read_Byte(MEAS_CFG_Addr);
		
		if(Sta != 0){SPL06_ID = Sta;return 0;}
		else{return 255;}
	}
	
	void Get_Value(float time_span){
		
		Config = SPL06_Read_Byte(MEAS_CFG_Addr);//判断数据是否准备完成
		if(((Config>>5)&0x01) && ((Config>>4)&0x01)){
				Pressure_MSB = SPL06_Read_Byte(PSR_B2_Addr);
				Pressure_CSB = SPL06_Read_Byte(PSR_B1_Addr);
				Pressure_LSB = SPL06_Read_Byte(PSR_B0_Addr);
				Temp_MSB = SPL06_Read_Byte(TMP_B2_Addr);
				Temp_CSB = SPL06_Read_Byte(TMP_B1_Addr);
				Temp_LSB = SPL06_Read_Byte(TMP_B0_Addr);
				pressure = (Pressure_MSB<<16)+(Pressure_CSB<<8)+Pressure_LSB;
				temperature = (Temp_MSB<<16)+(Temp_CSB<<8)+Temp_LSB;
		}
		last_height = height;
		altitude = 44330*(1-pow(Correcting_Pressure(Pressure_Para,Pressure_conversion(pressure,k_Press),Temperature_conversion(temperature,k_Temp))/101325.0f,1.0f/5.255f));
		height = BWF_barometer.f(altitude - init_altitude);
		z_speed_barometer= (height - last_height)/time_span;
	}
	
	uint32_t pressure,temperature;
	int Pressure_Para[7],Temperature_Para[2];
	float k_Press,k_Temp;
	float z_speed_barometer,altitude,init_altitude,last_height,height;
	uint8_t Config,Config_Press,Config_Temp;
	uint8_t Pressure_MSB,Pressure_CSB,Pressure_LSB;
	uint8_t Temp_MSB,Temp_CSB,Temp_LSB;
	
};


#endif
