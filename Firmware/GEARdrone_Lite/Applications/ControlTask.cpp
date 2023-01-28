/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @author  EXcai
  * @brief   无人机状态机
  *
  ==============================================================================
													 该文件包含以下内容
  ==============================================================================
    @note
			-无人机异常状态监控（低压、倾角过大、传感器故障等）
			-简易的无人机状态机
			-无人机状态调度任务

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

#include "Drone.h"

GearDrone Drone;
Mpu6050_Data Mpu6050;
extern UWB Uwb;
extern int volt[2];

/*光流数据处理*/
extern uint8_t optic_flow[9];
extern uint8_t data[100];

extern uint8_t remote_command;
MeanFilter<5> Power;
/********************************************************************************
 * @brief  	无人机异常状态监控
 
 * @param  	时刻准备进入异常模式
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Error_Monitor (uint8_t drone_state){
	
	if(Error_Code != 0){
		Drone_Mode = 5;
		drone_state = 0;
		return drone_state;
	}
	
	if(battery_voltage[1] < 375){
		Drone.Init_PWM = 0;
		Error_Code = 2;//low power
		drone_state = 0;
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);//电池电量低，关灯
		return drone_state;
	}
	if(fabsf(pitch) > 40 || pitch == 0){
		Error_Code = 3;//out of angle limit
		drone_state = 0;
		return drone_state;
	}
	if(fabsf(roll) > 40 || roll == 0){
		Error_Code = 3;//out of angle limit
		drone_state = 0;
		return drone_state;
	}
	if(Drone.z_pos > 2.6){
		Drone.Init_PWM = 0;
		Error_Code = 5;//超过限高
		drone_state = 0;
		return drone_state;
	}
	if(optic_flow[0] == 0){
		Error_Code = 6;//optical flow error
		drone_state = 0;
		return drone_state;
	}
	if(VL53.comms_type != 1){
		Error_Code = 7;//tof error
		drone_state = 0;
		return drone_state;
	}
	
	if(Mpu6050.mpu_status != 0){//mpu error
		Mpu6050.mpu_error_count++;
	}
	else{
		Mpu6050.mpu_error_count = 0;
	}
	if(Mpu6050.mpu_error_count > 4){
		Error_Code = 8;//mpu error
		drone_state = 0;
		return drone_state;
	}
	
	#if USE_UWB
	Uwb.time_span = (Drone.systemtime_10ms - Uwb.system_time)/10.0f;
	if(Uwb.time_span > 10){
		Error_Code = 9;//uwb offline
		drone_state = 0;
		return drone_state;
	}
	#endif
	
	if(remote_command == 3){
		Error_Code = 12;// manual crash
	}
	
	return drone_state;
}
/********************************************************************************
 * @brief  	无人机启动 0
 
 * @param  	Startup()以初始化无人机，
 
 * @retval 	0: mode
						1: mode
						5: mode
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Startup(uint8_t mode){
	
	if(systemtime_10ms == 100){printf_dma(&huart3,"ATE0\r\n");}
	if(systemtime_10ms == 150){printf_dma(&huart3,"AT+CWMODE=1\r\n");}
	if(systemtime_10ms == 200){printf_dma(&huart3,"AT+CWJAP=\"GEAR\",\"geardrone\"\r\n");}
	#if NETWORK_PROTOCOL
	if(systemtime_10ms == 300){printf_dma(&huart3,"AT+MQTTUSERCFG=0,1,\"MAV%u\",\"admin\",\"public\",0,0,\"\"\r\n",Uwb.id);}
	if(systemtime_10ms == 350){printf_dma(&huart3,"AT+MQTTCONN=0,\"192.168.0.120\",1883,0\r\n");}
	if(systemtime_10ms == 600){printf_dma(&huart3,"ATE0\r\n");}
	if(systemtime_10ms == 700){printf_dma(&huart3,"AT+MQTTSUB=0,\"%s\",0\r\n","Remote");}
	if(systemtime_10ms == 750){printf_dma(&huart3,"AT+MQTTSUB=0,\"%s\",0\r\n","MATLAB");}
	if(systemtime_10ms == 800){printf_dma(&huart3,"AT+MQTTSUB=0,\"MAV%u_RX\",0\r\n",Uwb.id);memset(data,0,100);}
	#else
	if(systemtime_10ms == 350){printf("AT+CIPSTART=\"UDP\",\"255.255.255.255\",12138,100%u\r\n",Uwb.id);}
	if(systemtime_10ms == 450){printf("AT+CIPMODE=1\r\n");}
	if(systemtime_10ms == 550){printf("AT+CIPSEND\r\n");}
	#endif
	if(systemtime_10ms < 800){
		battery_voltage[1] = battery_voltage[0];
		Mpu6050.x_speed_mpu = 0;
		Mpu6050.y_speed_mpu = 0;
		Mpu6050.z_speed_mpu = 0;
		yaw_init = yaw;
	}
	if(systemtime_10ms > 800 && Controller_State == 0){
		Controller_State = 1;
		mode = 1;
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);//LED on
		target_x_pos = init_x_pos;
		target_y_pos = init_y_pos;
	}
	return mode;
}
/********************************************************************************
 * @brief  	无人机起飞状态 1
 
 * @param  	无人机起飞状态
 
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Taskoff_State(uint8_t mode){
	
	if(Error_Code == 0 && remote_command == 2){Controller_State = 1;remote_command = 0;}
	
	if(Controller_State == 1){
			if(remote_command == 1){
				/*逐渐提升转速以起飞*/
				target_z_pos = 0.4f;
				if(z_pos < 0.095 && Init_PWM < 9500){
					if(Init_PWM < 4000){Init_PWM = 8100;}
					Init_PWM += 10;
				}
				else{
					mode = 2;
				}
				PID_position_controller(Controller_State);
			}
			else{
				#if DISABLE_MOTORS //电机不要转动
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);//lb
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);//rb
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,0);//lf
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,0);//rf
				#else
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,Init_PWM);//lb
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,Init_PWM);//rb
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,Init_PWM);//lf
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,Init_PWM);//rf
				#endif
			}
		}
		else{
			mode = 5;
			Error_Code = 11;
		}
		return mode;
}

/********************************************************************************
 * @brief  	无人机遥控器控制模式 2
 
 * @param  	遥控器控制模式
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Remote_State(uint8_t mode){
	
	if(Controller_State == 1 && mode == 2){
	/********************************************************************************
	* @brief  生成并使用机内绕圆轨迹
	*
	* @param  		UAV1
	*
	*					UAV2		UAV4
	*
	*				  		UAV3
	*
	* @author EXcai亢体造梦
	********************************************************************************/
	
	#if RUN_CIRCLE_PATH//生成并使用机内绕圆轨迹
	static uint16_t action_start_time;
	if(z_pos > target_z_pos - 0.1 && remote_command == 1 && Controller_State == 1){
		if(action_start_time == 0){action_start_time = systemtime_10ms;}
		if(systemtime_10ms - action_start_time > 300){
			if(Uwb.id == 1){
				target_x_pos = RADIUS*(sinf(((float)(systemtime_10ms - action_start_time - 300))*T*WR)) + CENTER_X_POS;
				target_y_pos = RADIUS*(cosf(((float)(systemtime_10ms - action_start_time - 300))*T*WR)) + CENTER_Y_POS;
			}
			else if(Uwb.id == 2){
				target_x_pos = RADIUS*(sinf(((float)(systemtime_10ms - action_start_time - 300))*T*WR + PI*1/2)) + CENTER_X_POS;
				target_y_pos = RADIUS*(cosf(((float)(systemtime_10ms - action_start_time - 300))*T*WR + PI*1/2)) + CENTER_Y_POS;
			}
			else if(Uwb.id == 3){
				target_x_pos = RADIUS*(sinf(((float)(systemtime_10ms - action_start_time - 300))*T*WR + PI)) + CENTER_X_POS;
				target_y_pos = RADIUS*(cosf(((float)(systemtime_10ms - action_start_time - 300))*T*WR + PI)) + CENTER_Y_POS;
			}
			else if(Uwb.id == 4){
				target_x_pos = RADIUS*(sinf(((float)(systemtime_10ms - action_start_time - 300))*T*WR + PI*3/2)) + CENTER_X_POS;
				target_y_pos = RADIUS*(cosf(((float)(systemtime_10ms - action_start_time - 300))*T*WR + PI*3/2)) + CENTER_Y_POS;
			}
		}
	}
	else{
		target_x_pos = init_x_pos;
		target_y_pos = init_y_pos;
	}
	#endif
	
	/*动态高度*/
	#if (DYNAMIC_HEIGHT==1)
		if(z_pos > 0.3 && z_pos < HEIGHT_TARGET){
			target_z_pos += 0.0015;
		}
	#endif
	
	/*切换控制器*/
	#if OUTPUT_REGULATION_CONTROLLER
	OR_position_controller(Controller_State,0.01f);
	#else
	PID_position_controller(Controller_State);
	#endif	
	
	}
	if(remote_command == 2){
		mode = 4;
	}
	return mode;
}
/********************************************************************************
 * @brief  	无人机编队控制模式 3
 
 * @param  	这里写编队控制算法，设置目标位置 速度 加速度
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Formation_State(uint8_t mode){
	if(Controller_State == 1 && mode == 3){
		OR_position_controller(Controller_State,0.01f);
	}
	if(remote_command == 2){
		mode = 4;
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
	static uint16_t suspend_time;
	if(Controller_State == 1 && mode == 4){
		if(z_pos > 0.8){target_z_pos = (target_z_pos - 0.001f);}
		else{
			Init_PWM = 9500;
			if(suspend_time == 0){suspend_time = systemtime_10ms;target_z_pos = z_pos;}
			if(systemtime_10ms - suspend_time > 300){target_z_pos = (target_z_pos - 0.001f);}
		}
		PID_position_controller(Controller_State);
		if(z_pos < 0.1f){
			mode = 1;
			Controller_State = 0;
			Init_PWM = 0;
		}
	}
	return mode;
}

uint8_t GearDrone::Crash_State(uint8_t mode){
	Controller_State = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);//lb
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);//rb
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,0);//lf
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,0);//rf
	return mode;
}

void Control_Task(void const * argument)
{
  /* USER CODE BEGIN StartGoTask */
	Drone.systemtime_10ms = 0;
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
		taskENTER_CRITICAL();//临界区
		/* 获取系统时间 */
		Drone.systemtime_10ms = Get_sys_time_ms()/10;
		
		/* 电压记录 */
		Power << ((volt[0] * 2) * 3.3f / 40.96f);
		Power >> Drone.battery_voltage[0]; 
		
		/*读取陀螺仪*/
		Drone.last_yaw = Drone.yaw; 
		Mpu6050.mpu_status = mpu_dmp_get_data(&Drone.pitch,&Drone.roll,&Drone.yaw);
		
		Drone.yaw += Drone.cycles*360;
		if((Drone.yaw - Drone.last_yaw) > 180){Drone.yaw -= 360.0f;Drone.cycles--;}
		else if((Drone.yaw - Drone.last_yaw) < -180){Drone.yaw += 360.0f;Drone.cycles++;}
		
		/*异常状态监控*/
		if(Drone.systemtime_10ms > 600){
			Drone.Controller_State = Drone.Error_Monitor(Drone.Controller_State);
		}	
		
		taskEXIT_CRITICAL();//退出临界区
		/*状态机和控制任务*/
		switch(Drone.Drone_Mode){
			/* 以下部分用于简易状态机 */
			case 0:
				Drone.Drone_Mode = Drone.Startup(Drone.Drone_Mode);
				break;
			case 1:
				Drone.Drone_Mode = Drone.Taskoff_State(Drone.Drone_Mode);
				break;
			case 2:
				Drone.Drone_Mode = Drone.Remote_State(Drone.Drone_Mode);
				break;
			case 3:
				Drone.Drone_Mode = Drone.Formation_State(Drone.Drone_Mode);
				break;
			case 4:
				Drone.Drone_Mode = Drone.Landing_State(Drone.Drone_Mode);
				break;
			case 5:
				Drone.Drone_Mode = Drone.Crash_State(Drone.Drone_Mode);
				break;
//			case 6:
//				Drone.Drone_Mode = Drone.Debug_State(Drone.Drone_Mode);
//				break;
			default:break;
		}
    osDelayUntil(&xLastWakeTime,xFrequency);
  }
  /* USER CODE END StartGoTask */
}
