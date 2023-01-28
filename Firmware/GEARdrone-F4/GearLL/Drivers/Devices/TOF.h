/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    TOF.h
  * @author  EXcai
  * @brief   激光测距GearLL驱动
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
#pragma once
	
#ifndef __TOF_H__
#define __TOF_H__

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "Sensors.h"
#include "math.h"
#include "vl53l0x.h"
#include "vl53l0x_gen.h"
#include "i2c_def.h"


extern VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);
extern VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);
extern VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf);
extern VL53L0X_RangingMeasurementData_t vl53l0x_data;//测距测量结构体
extern VL53L0X_Dev_t vl53l0x_dev;//设备I2C数据参数
extern uint16_t Distance_data;//保存测距数据
static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
extern VL53L0X_Error Status;//工作状态

extern uint32_t Get_SystemTimer(void);

/* Variables -------------------------------------------------------------------*/

static MeanFilter<3> MF_tof;
static MedianFilter<5> MDF_tof_speed;

struct tof_data{
	
	uint8_t Status;
	uint32_t timestamp_ms;
	
	float TOF_Height,Last_TOF_Height,TOF_Speed;
};

/** 
* @brief Class for VL53L0X.
*/
class VL53L0X : public Sensor<tof_data>{
public:
	VL53L0X(){};
	virtual ~VL53L0X(){};
	uint8_t VL53L0X_ID;
	uint8_t Status;

	virtual uint8_t Init(){
		
		uint8_t Sta;
		
		TOF_I2C_Init();
		Sta = vl53l0x_init(&vl53l0x_dev);
		Sta = vl53l0x_set_mode(&vl53l0x_dev,3);
		if (Status == VL53L0X_ERROR_NONE){
			Status = VL53L0X_SetLimitCheckValue(&vl53l0x_dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.25*65536));
    }
		if (Status == VL53L0X_ERROR_NONE){
			Status = VL53L0X_SetLimitCheckValue(&vl53l0x_dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(32*65536));
    }
		if (Status == VL53L0X_ERROR_NONE){
			Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53l0x_dev,20000);
    }
		return Sta;
	}
	
	virtual uint8_t Calibration(){return 0;}
	virtual bool Calibration_Status(){return 0;}
	virtual struct tof_data Get_Value(){

		tof_data data;
		
		Status = vl53l0x_interrupt_start(&vl53l0x_dev,&vl53l0x_data,buf);
		
		Speed_catch[5] = Speed_catch[4];
		Speed_catch[4] = Speed_catch[3];
		Speed_catch[3] = Speed_catch[2];
		Speed_catch[2] = Speed_catch[1];
		Speed_catch[1] = Speed_catch[0];
		MF_tof << Distance_data;
		MF_tof >> Speed_catch[0];
		
		if(fabsf((Speed_catch[0] - Speed_catch[5])/0.1f) < 500){
			MDF_tof_speed << (Speed_catch[0] - Speed_catch[5])/0.1f;//mm/s
			MDF_tof_speed >> TOF_Speed;
		}
		
		Last_TOF_Height = TOF_Height;
		TOF_Height = Speed_catch[0]/1000.0f;
		
		
		data.Last_TOF_Height = Last_TOF_Height;
		data.TOF_Height = TOF_Height;
		data.TOF_Speed = TOF_Speed;
		data.Status = Status;
		
		return data;
	}
	


	float TOF_Height,Last_TOF_Height,TOF_Speed;
	
	#ifndef USE_UPPERMONITOR
	private:
	#endif
	
	float Speed_catch[6];
	
};


#endif
