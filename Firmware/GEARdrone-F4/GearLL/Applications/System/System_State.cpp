/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    System_State.cpp
  * @author  EXcai
  * @brief   无人机各种状态
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
		- 参考
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
#include "Drone.h"
#include "Serial_Remote.h"
/* Private variables --------------------------------------------------------*/
MeanFilter<25> F_takeoff;
extern Light LED;
/* function prototypes -------------------------------------------------------*/

/********************************************************************************
 * @brief  	无人机调试模式 6
 
 * @param  	调试模式
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Debug_State(uint8_t mode){
	
	if(System_time_ms > _CONTROL_START_TIME && System_time_ms < _CONTROL_LIMIT_TIME){
//		target.pitch_target = 0;
//		target.roll_target = 0;
//		target.yaw_target = 0;
		
		Controller_State = 2;//0:控制失效;1:所有控制启动;2:只进行姿态控制;3:只进行高度+姿态控制
		mode = 6;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
	else if(System_time_ms > _CONTROL_LIMIT_TIME){
		Init_PWM = 0;
		Controller_State = 0;
		mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
	
	return mode;
	
}

/********************************************************************************
 * @brief  	无人机异常 5
 
 * @param  	异常处理
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Crash_State(uint8_t mode){
	
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);//LED
	Controller_State = 0;//0:控制失效;1:所有控制启动;2:只进行姿态控制;3:只进行高度+姿态控制
	Init_PWM = 0;
	
	return mode;
	
}

/********************************************************************************
 * @brief  	无人机启动函数 0
 
 * @param  	Startup()以初始化无人机，
 
 * @retval 	0: mode
						1: mode
						5: mode
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Startup(uint8_t mode){
	
	/*传感器初始化失败*/
	if(Sensors_State != 0){
		mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
	
	/*提前1000ms打开旋翼(如果define了NO_MOTOR，此条不生效)*/
	if(System_time_ms > _CONTROL_START_TIME - 1000){
		Init_PWM = 2000;
	}
	/*前10秒用于mpu6050和光流的稳定收敛到正确值*/
	if(System_time_ms < _CONTROL_START_TIME){
		return mode;
	}
	
	/*等待传感器校准,ESP ready?*/
	if(Qmc5883l.Calibration_state == 1 || Battery_Voltage.Calibration_state == 1 || Esp.Init_State != 2) {
		return mode;
	}
#ifdef DEBUG_MODE
	mode = 6;
#else
	
	if(Remote_Mode == 0 || Remote_Mode == 4){
		Init_PWM = 0;
	}
	
	/*判断起飞条件*/
	if(init_data.VBAT > 3.85f && Remote_Mode == 2 && System_time_ms > _CONTROL_START_TIME){
		mode = 1;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
	else if(init_data.VBAT <= 3.85f){
		mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
#endif	
	return mode;
	
}

/********************************************************************************
 * @brief  	无人机起飞状态 1
 
 * @param  	无人机起飞状态
 
 * @retval 	0: mode
						1: mode
						5: mode
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Taskoff_State(uint8_t mode){
	
	float temp_height;
	LED.Task(System_time_us,1);
	
	/*一个巨长的滤波器，用来判断是不是在目标高度附近浮动*/
	F_takeoff << current_data.pos_z;
	F_takeoff >> temp_height;
	
	target.pos_z_target = _TARGET_HEIGHT;//目标高度
	
	if(current_data.pos_z > _LIMIT_HEIGHT){//高度超过_LIMIT_HEIGHT 紧急停机
		mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		return mode;
	}
	
	/*判断升空条件*/
	if(current_data.pos_z < 0.1f){//定点切换判断
		if(current_data.pos_z < 0.06f){//判断是否在地上
			Controller_State = 2;//2：只进行姿态控制
			if(Init_PWM < 8000){
				Init_PWM = 4000;
				Init_PWM += 10;
			}
		}
		else{//判断是否贴地
			Controller_State = 3;//3:只进行高度+姿态控制
		}
		Flow.sum_flow_x = 0;
		Flow.sum_flow_y = 0;
	}
	else{
		Controller_State = 1;//1:所有控制启动
		mode = 2;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
	return mode;
	
}

/********************************************************************************
 * @brief  	无人机遥控器控制模式 2
 
 * @param  	遥控器控制模式
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
extern Serial_Remote Remote;
extern struct serial_data serial_remote;
uint8_t GearDrone::Remote_State(uint8_t mode){
	
	LED.Task(System_time_us,2);
	
	if(Remote_Mode == 0){//遥控器离线
		mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
	if(Remote_Mode == 2){
		mode = 2;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
	}
	#ifndef DEBUG_MODE
	else if(Remote_Mode == 3){//formation mode
		mode = 3;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		//...
	}
	else if(Remote_Mode == 4){//landing
		mode = 4;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		//...
	}
//	else if(target.pos_z_target < 0.25f){
//		Drone_Mode = 4;
//	}
	#endif
	return mode;
}

/********************************************************************************
 * @brief  	无人机编队控制模式 3
 
 * @param  	这里写编队控制算法，设置目标位置 速度 加速度
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Formation_State(uint8_t mode){
	
	LED.Task(System_time_us,3);
	
	if(Remote_Mode == 2){//remote mode
		mode = 2;
	}
	else if(1){
		mode = 4;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		//...
	}
	
	return mode;
	
}

/********************************************************************************
 * @brief  	无人机降落模式 4
 
 * @param  	降落模式
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Landing_State(uint8_t mode){
	
	LED.Task(System_time_us,4);
	
	if(Init_PWM >= 3500){
//		Controller_State = 2;//只进行姿态控制
//		Height_PID.Out = 0;
//		Height_Speed_PID.Out = 0;
//		Init_PWM -= 150;
		Controller_State = 0;//0:控制失效
		Init_PWM = 0;
		mode = 5;
	}
	else if(current_data.pos_z < 0.2f){
		Controller_State = 0;//0:控制失效
		Init_PWM = 0;
		mode = 5;
	}
	
	return mode;
	
}

/********************************************************************************
 * @brief  	无人机状态监视器
 
 * @param  	时刻准备进入异常模式
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Error_Monitor(){
	
	if(Drone_Mode != 0 && Drone_Mode != 5){
		
		/*角度超过阈值*/
		if(fabsf(current_data.pitch) > 35 || fabsf(current_data.roll) > 35){
			Drone_Mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		}
		/*MPU6050抽风了*/
		if(current_data.pitch == 0 || current_data.roll == 0){
			Drone_Mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		}
		
		/*高度超过阈值*/
		if(current_data.pos_z > _LIMIT_HEIGHT){
			Drone_Mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		}
	}
	
#ifndef DEBUG_MODE
	if(Esp.heart != 0 && Esp.Init_State == 2){//遥控器在线过
		if(Remote_Mode == 0){//遥控器掉线
			Drone_Mode = 5;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		}
		if(Remote_Mode == 4){//landing
			Drone_Mode = 4;//0初始化 1起飞 2遥控器控制 3编队控制 4降落 5紧急停机 6调试模式
		}
	}
#endif
	
	return 0;
	
}

