/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Drone.h
  * @author  EXcai
  * @brief   无人机类，包含各种抽象操作和状态
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
			-懒得写了

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

  ********************************************************************************/

#ifndef _DRONE_H_
#define _DRONE_H_

#include "main.h"
#include "cmsis_os.h"
#include "System_Config.h"

#ifdef __cplusplus    
extern "C" {         
#endif
	
#include "i2c.h"

#if (TOF_SENSOR==0)
#include "vl53l0x.h"
#include "vl53l0x_gen.h"
#endif

#include "mpu6050.h"

extern uint32_t Get_sys_time_ms(void);
extern void Control_Task(void const * argument);
extern void Data_Task(void const * argument);
extern void Upper_Monitor(void const * argument);
extern void TOF_Task(void const * argument);
extern void Uwbunpack(void const * argument);
extern void Flowunpack(void const * argument);
extern void Mpuunpack(void const * argument);
#ifdef __cplusplus
}
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#include "task.h"
#include "math.h"
#include "filter.h"

#include "PID.h"
#include "UWB.h"
#include "inv_mpu.h"

#include "Butterworth.h"
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* Semaphore */
extern xSemaphoreHandle uwb_Semaphore;
extern xSemaphoreHandle flow_Semaphore;
extern xSemaphoreHandle esp_Semaphore;

#if USE_MAGSENSOR
#include "HMC5883L.h"
extern float flash_params[9];
extern HMC5883L Magsensor;
extern mag_data mag;

/* 二阶巴特沃斯低通滤波器系数*/ 
static float IIRCoeffs32LP[5] = {1.0f, 2.0f, 1.0f, 1.143f,-0.413f};
ButterworthFilter BWF_mag(IIRCoeffs32LP,0.0674f,1);
#endif

#if USE_BAROMETER
#include "SPL06_001.h"
extern SPL06 Pressure_sensor;
#endif

#if (TOF_SENSOR==0)
extern VL53L0X_Dev_t vl53l0x_dev;
extern VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);
extern VL53L0X_RangingMeasurementData_t vl53l0x_data;
extern uint16_t Distance_data;
static char buf[VL53L0X_MAX_STRING_LENGTH];
VL53L0X_Error Status=VL53L0X_ERROR_NONE;
#else
#include "vl53l1.h"
extern int32_t tof_distance_int32;
extern VL53L1_Dev_t VL53;
extern VL53L1_RangingMeasurementData_t result_data;
#endif

/** 
* @brief Class for drone.
*/
class GearDrone{
	public:
		
		GearDrone(){};
		~GearDrone(){};					

		uint8_t Startup(uint8_t mode);
			
		uint8_t Taskoff_State(uint8_t mode);

		uint8_t Remote_State(uint8_t mode);

		uint8_t Formation_State(uint8_t mode);

		uint8_t Landing_State(uint8_t mode);

		uint8_t Crash_State(uint8_t mode);
		
		uint8_t Debug_State(uint8_t mode);
				
		uint8_t Drone_Mode;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		
		uint8_t Controller_State;//0:控制失效;1:所有控制启动;
			
		uint8_t Error_Monitor (uint8_t drone_state);
		uint8_t Error_Code;
		
		uint16_t LF_PWM,LB_PWM,RF_PWM,RB_PWM,Init_PWM;
		uint32_t systemtime_10ms,network_time;
		int16_t cycles;//yaw循环数
		
		void OR_position_controller(uint8_t controller_state,float period);
		void PID_position_controller(uint8_t controller_state);
		void PID_velocity_controller(float x_target, float y_target, float z_target);

		void angular_velocity_controller(uint8_t controller_state);
		void angle_controller(void);
		
		float pitch,roll,yaw,last_yaw,yaw_init;
		float pitch_speed,roll_speed,yaw_speed;

		float x_pos,y_pos,z_pos;
		float x_vel,y_vel,z_vel;
		float x_acc,y_acc,z_acc;
		float init_x_pos,init_y_pos;
		
		float target_x_pos,target_y_pos,target_z_pos;
		float target_x_vel,target_y_vel,target_z_vel;
		float yaw_target_angle;
		float acc_target[3];//x(roll) y(pitch) z三轴加速度的目标值(仅在matlab二阶控制模式下有效)
		
		float battery_voltage[2];
			
		myPIDTimer Timer1;
		myPID Yaw_PID,Pitch_PID,Roll_PID,Pitch_Speed_PID,Roll_Speed_PID,Yaw_Speed_PID;
		myPID Speed_x_PID, Speed_y_PID, Speed_z_PID, Pos_x_PID, Pos_y_PID, Pos_z_PID;
};

#endif

#endif
