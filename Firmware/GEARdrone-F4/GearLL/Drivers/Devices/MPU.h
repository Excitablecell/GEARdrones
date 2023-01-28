/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    MPU.h
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
	
#ifndef __MPU_H__
#define __MPU_H__

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "Sensors.h"
#include "math.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "i2c_def.h"

/* Defines ------------------------------------------------------------------*/

#ifndef PI     //如果PI没有被定义 
#define PI 3.1415926 
#endif

#ifndef USE_SOFT_IIC
extern I2C_HandleTypeDef hi2c1;
#endif

/* Variables -------------------------------------------------------------------*/

static MeanFilter<7> MDF_x_acc,MDF_y_acc,MDF_z_acc;
static MeanFilter<3> MF_pitch_speed,MF_roll_speed;

struct imu_data{
	
	uint8_t Status;
	uint32_t timestamp_ms;
	
	float pitch,roll,yaw,yaw_raw;
	int16_t circle;
	float pitch_speed,roll_speed,yaw_speed;
	
	float x_acc,y_acc,z_acc;
	float mpu_x_v,mpu_y_v,mpu_z_v;
};

/** 
* @brief Class for MPU6050.
*/
class MPU6050 : public Sensor<imu_data>{
public:
	MPU6050(){};
	virtual ~MPU6050(){};
	uint8_t MPU6050_Status;
	uint8_t DMP_Init_Status;

	virtual uint8_t Init(){
		
		uint8_t Sta;
		
		HAL_Delay(10);
		MPU6050_Init();
		HAL_Delay(10);
		DMP_Init_Status = mpu_dmp_init();
//		HAL_Delay(10);
//		MPU_Write_Byte(MPU_CFG_REG,0x03);// 设置MPU6050	内部DLPF滤波器截止频率42HZ 滞后4.8ms
		return Sta;
	}
	
	virtual uint8_t Calibration(){return 0;}
	virtual bool Calibration_Status(){return 0;}
	virtual struct imu_data Get_Value(){
		imu_data data;
		/*读取陀螺仪任务*/
		last_yaw = yaw; 
		MPU6050_Status = mpu_dmp_get_data(&pitch_raw,&roll_raw,&yaw_raw);
		if(MPU6050_Status != 0 && (pitch_raw == 0 || roll_raw == 0)){
			MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);//reboot
			HAL_Delay(50);
		}
		data.Status = MPU6050_Status;
		
		/*保证机头向前俯视下，抬头为pitch增，右倾斜转体为roll增，顺时针转为yaw增*/
		pitch = roll_raw;//20220411 需要抬头为pitch增
		roll = pitch_raw+1;//20220411 需要右倾斜转体为roll增
		yaw = -yaw_raw;//20220411 需要顺时针转为yaw增
		
		yaw += circle*360;
		if((yaw - last_yaw) > 180){yaw -= 360.0f;circle--;}
		else if((yaw - last_yaw) < -180){yaw += 360.0f;circle++;}
		
		/*读取陀螺仪角速度任务*/
		MPU_Get_Gyroscope(gyro);
		MF_pitch_speed << gyro[0]/16.4;
		MF_pitch_speed >> pitch_speed;
		MF_roll_speed << gyro[1]/16.4;
		MF_roll_speed >> roll_speed;
		yaw_speed = -gyro[2]/16.4;
		
		/*读取加速度*/
		MPU_Get_Accelerometer(acc);
		/*Roll*/MDF_x_acc << cosf((-1.0f*yaw*PI)/180.0f)*arm_cos_f32((roll*PI)/180.0f)*(acc[0]/16384.0f)
										+ (arm_sin_f32((pitch*PI)/180.0f)*arm_sin_f32((roll*PI)/180.0f)*cosf((-1.0f*yaw*PI)/180.0f) - arm_cos_f32((pitch*PI)/180.0f)*sinf((-1.0f*yaw*PI)/180.0f))*(acc[1]/16384.0f)
										+ (arm_sin_f32((pitch*PI)/180.0f) * arm_sin_f32((-1*yaw*PI)/180.0f) + arm_cos_f32((pitch*PI)/180.0f) * arm_sin_f32((roll*PI)/180.0f) * cosf((-1*yaw*PI)/180.0f))*(acc[2]/16384.0f);
		
		/*Pitch*/MDF_y_acc << sinf((-1.0f*yaw*PI)/180.0f)*arm_cos_f32((roll*PI)/180.0f)*(acc[0]/16384.0f)
		                 + (arm_sin_f32((pitch*PI)/180.0f)*arm_sin_f32((roll*PI)/180.0f)*sinf((-1*yaw*PI)/180.0f) + arm_cos_f32((pitch*PI)/180.0f)*cosf((-1.0f*yaw*PI)/180.0f))*(acc[1]/16384.0f)
										 + (arm_cos_f32((pitch*PI)/180.0f) * sinf((-1.0f*yaw*PI)/180.0f) * arm_sin_f32((roll*PI)/180.0f) - arm_sin_f32((pitch*PI)/180.0f) * cosf((-1.0f*yaw*PI)/180.0f))*(acc[2]/16384.0f);
		
		/*Z*/MDF_z_acc << - 1*arm_sin_f32((roll*PI)/180.0f)*(acc[0]/16384.0f)
								 + arm_sin_f32((pitch*PI)/180.0f)*arm_cos_f32((roll*PI)/180.0f)*(acc[1]/16384.0f)
								 + arm_cos_f32((pitch*PI)/180.0f)*arm_cos_f32((roll*PI)/180.0f)*(acc[2]/16384.0f)
								 - 1.09f;
			
			MDF_x_acc >> x_acc;
			MDF_y_acc >> y_acc;
			MDF_z_acc >> z_acc;
				
//		if(fabsf(x_acc) > 0.07f){
//			mpu_x_v += x_acc;
//		}
//		else if(fabsf(x_acc) < 0.07f && fabsf(x_acc) > 0.03f){
//			mpu_x_v -= x_acc;
//		}
//		
//		if(fabsf(y_acc) > 0.07f){
//			mpu_y_v += y_acc;
//		}
//		else if(fabsf(y_acc) < 0.07f && fabsf(x_acc) > 0.03f){
//			mpu_y_v -= y_acc;
//		}
//		
//		if(fabsf(z_acc) > 0.07f){
//			mpu_z_v += z_acc;
//		}
//		else if(fabsf(z_acc) < 0.07f && fabsf(x_acc) > 0.03f){
//			mpu_z_v -= z_acc;
//		}
		
		
		/*传至GEARdrone对象*/
		data.timestamp_ms = System_time_us/1000;
		data.pitch = pitch;
		data.roll = roll;
		data.yaw = yaw;
		data.yaw_raw = yaw_raw;
		data.circle = circle;
		
		data.pitch_speed = pitch_speed;
		data.yaw_speed = yaw_speed;
		data.roll_speed = roll_speed;
		
		data.x_acc = x_acc;
		data.y_acc = y_acc;
		data.z_acc = z_acc;
		
		data.mpu_x_v = mpu_x_v;
		data.mpu_y_v = mpu_y_v;
		data.mpu_z_v = mpu_z_v;
		
		return data;
	}
	

	int16_t circle;
	float pitch,roll,yaw;
	float pitch_speed,roll_speed,yaw_speed;
	float last_yaw;

	short gyro[3],acc[3];
	float x_acc,y_acc,z_acc;
	float pitch_raw,roll_raw,yaw_raw;
	float pitch_offset,roll_offset,yaw_offset;
	float mpu_x_v,mpu_y_v,mpu_z_v;
	
	#ifndef USE_UPPERMONITOR
	private:
	#endif
};


#endif
