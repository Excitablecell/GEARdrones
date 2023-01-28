/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Drone.h
  * @author  EXcai
  * @brief   无人机类，包含各种抽象操作和状态
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
		- 不会请参考HMC5883L:

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
**/
#ifndef _DRONE_H_
#define _DRONE_H_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
#include <stdint.h>
#include <stddef.h>
#include <limits.h>

#include "main.h"
#include "cmsis_os.h"

#include "filter.h"
#include "System_Startup.h"

#include "Light.h"

/* Private variables ---------------------------------------------------------*/

/** 
* @brief 无人机状态结构体
*/

/** 
* @brief 用于机间通讯的数据结构体
*/
 
struct Drone_public_data
{

	float pos_x,pos_y,pos_z;
	float x_speed,y_speed,z_speed;
	
	float distance[5],velocity[5];
	uint8_t id;
};

struct Drone_data
{
	uint32_t timestamp_ms,network_time;

	float pitch,roll,yaw,imu_yaw;
	float pitch_speed,roll_speed,yaw_speed;

	float pos_x,pos_y,pos_z;
	float x_speed,y_speed,z_speed;
	float x_acc,y_acc,z_acc;
	
	float altitude,altitude_speed,stoo_distance[5],stoo_velocity[5];//stoo:self to others
	
	float VBAT;
};

/** 
* @brief 无人机指令目标数据结构体
*/

struct Drone_target
{
	uint32_t timestamp_ms;

	float pitch_target,roll_target,yaw_target;
	float pitch_speed_target,roll_speed_target,yaw_speed_target;

	float pos_x_target,pos_y_target,pos_z_target;
	float speed_x_target,speed_y_target,speed_z_target;
};

/** 
* @brief Class for sensors.
*/
class GearDrone{
	public:
		
		GearDrone(){};
		~GearDrone(){};					
			
			
		uint8_t Get_Sensors_Data(pressure_data *Spl_temp,mag_data *Mag_temp,tof_data *Tof_temp,flow_data *Flow_temp,imu_data *Imu_temp,adc_data *Adc_temp,uwb_struct_data *Uwb_temp);
		
		uint8_t Fusion_Height(uint8_t mode,pressure_data *altitude,tof_data *height,imu_data *mpu);
		uint8_t Fusion_Mov(uint8_t mode,imu_data *mpu,flow_data *flow,uwb_struct_data *Uwb_temp);

		uint8_t Controller(uint8_t status);
			
		uint8_t NLR_Get_Data(float uwb_distances[7]);
		
		uint8_t Communication(uwb_struct_data *Uwb_temp);
		
		uint8_t Error_Monitor();
			
		uint8_t Basic_Dynamic_Model();
		uint8_t Advanced_Dynamic_Model();
		


		uint8_t Startup(uint8_t mode);
			
		uint8_t Taskoff_State(uint8_t mode);

		uint8_t Remote_State(uint8_t mode);

		uint8_t Formation_State(uint8_t mode);

		uint8_t Landing_State(uint8_t mode);

		uint8_t Crash_State(uint8_t mode);
		
		uint8_t Debug_State(uint8_t mode);
				
		uint8_t Drone_Mode;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		
		uint8_t Sensors_State;// 0:succes; others:error
		
		uint8_t Remote_Mode;// 0:offline 1: online 2:allow take-off 3:formation mode 4: landing
		
		uint8_t Controller_State;//0:控制失效;1:所有控制启动;2:只进行姿态控制;3:只进行高度+姿态控制
		
		uint8_t Reboot_Reason;

		float uwb_d[7];

		uint8_t UWB_ID,Flash_ID;
		uint32_t System_time_ms,Flying_time_ms,period_beat;
		float real_time;

		
		struct Drone_data current_data,last_data,init_data;
		struct Drone_public_data public_data,others_public_data[5];
		struct Drone_target target;
		uint16_t LF_PWM,LB_PWM,RF_PWM,RB_PWM,Init_PWM;
		
		myPID Yaw_PID,Pitch_PID,Roll_PID,Pitch_Speed_PID,Roll_Speed_PID,Yaw_Speed_PID;
		myPID Speed_X_PID, Speed_Y_PID,Pos_X_PID, Pos_Y_PID;
		myPID Height_PID,Height_Speed_PID;
		
#ifndef USE_UPPERMONITOR
	protected:
#endif		
		float pitch_speed_wait,roll_speed_wait;
};

#endif

#endif
