/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    HMC5883L.h
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
#ifndef __HMC5883L_H_
#define __HMC5883L_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "math.h"
#include "Drone.h"

/* Variables -------------------------------------------------------------------*/

struct mag_data{
	
	float yaw;
	
	int16_t X_HM,Y_HM,Z_HM;
};

#ifdef __cplusplus    
extern "C" {         
#endif
	
extern void FlashWriteBuff( const uint32_t destination_address, uint8_t *const buffer,uint32_t length );
extern void FlashReadBuff(const uint32_t source_address,uint8_t *const buffer,uint16_t length);
	
#ifdef __cplusplus
}
#endif
	
void HMC5883L_ReadAngle(int16_t *x,int16_t *y,int16_t *z);
uint8_t HMC5883L_Init(void);
uint8_t QMC_Data_Status(void);
/** 
* @brief Class for HMC5883L.
*/
class HMC5883L{
public:
	HMC5883L(){};
	~HMC5883L(){};
	uint8_t HMC5883L_ID;

	uint8_t Init(int16_t *offset_address){
		HMC5883L_ID = HMC5883L_Init();
		FlashReadBuff(0x8014000,(uint8_t *)offset_address,36);  // 从Flash中读取数
		memcpy(Axis_offset,&offset_address[12],sizeof(Axis_offset));
		if(HMC5883L_ID == 0xff){return 0;}
		else{return 1;}
	}
	
	uint8_t Calibration(int16_t *offset_address, uint16_t time){
		if(time < 4000){
			HMC5883L_ReadAngle(&X_HM,&Y_HM,&Z_HM);
			if(Axis_max[0] < X_HM){Axis_max[0] = X_HM;}
			if(Axis_min[0] > X_HM){Axis_min[0] = X_HM;}
			if(Axis_max[1] < Y_HM){Axis_max[1] = Y_HM;}
			if(Axis_min[1] > Y_HM){Axis_min[1] = Y_HM;}
			if(Axis_max[2] < Z_HM){Axis_max[2] = Z_HM;}
			if(Axis_min[2] > Z_HM){Axis_min[2] = Z_HM;}
			Axis_offset[0] = (Axis_max[0] + Axis_min[0])/2;
			Axis_offset[1] = (Axis_max[1] + Axis_min[1])/2;
			Axis_offset[2] = (Axis_max[2] + Axis_min[2])/2;
			Axis_offset[3] = (Axis_max[0] - Axis_min[0]);
			Axis_offset[4] = (Axis_max[1] - Axis_min[1]);
			Axis_offset[5] = (Axis_max[2] - Axis_min[2]);
			memcpy(&offset_address[12],Axis_offset,sizeof(Axis_offset));
		}
		else{
			FlashWriteBuff(0x8014000,(uint8_t *)offset_address,36);  // 写入数据到Flash
			return 0;
		}
		return 1;
	}
	struct mag_data Get_Value(){
		
		mag_data data;
		
		//Status = QMC_Data_Status();
		
		HMC5883L_ReadAngle(&X_temp,&Y_temp,&Z_temp);
		if(X_temp != (int16_t)0xffff && Y_temp != (int16_t)0xffff && Z_temp != (int16_t)0xffff){X_HM = X_temp;Y_HM = Y_temp;Z_HM = Z_temp;}//保证机头向前俯视下，XH像前指，YH向右指，以此为标准确定XH、YH对于的真实XY（即虚拟XHYH和芯片XY）的关系
		
		data.X_HM = ((X_HM - Axis_offset[0])*1000.0f/Axis_offset[3]);//保证机头向前俯视下，XH像前指，YH向右指，以此为标准确定XH、YH对于的真实XY（即虚拟XHYH和芯片XY）的关系
		data.Y_HM = ((Y_HM - Axis_offset[1])*1000.0f/Axis_offset[4]);
		data.Z_HM = ((Z_HM - Axis_offset[2])*1000.0f/Axis_offset[5]);
		
		XVA = data.X_HM;
		YVA = data.Y_HM;
		ZVA = data.Z_HM;
		return data;
	}
	uint8_t Status;
	int16_t X_temp,Y_temp,Z_temp;
	int16_t X_HM,Y_HM,Z_HM;
	float XVA,YVA,ZVA;
	int16_t Axis_offset[6],Axis_min[3],Axis_max[3];//0~2顺序：x y z轴 Axis_offset[6]分别是xyz的offset和xyz的范围（用于归一化）

};

#endif
