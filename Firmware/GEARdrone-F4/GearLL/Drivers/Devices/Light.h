/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Light.h
  * @author  EXcai
  * @brief   信号灯驱动
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
#include "Drone.h"

class Light{
public:
	Light(){};
	~Light(){};
	
	/********************************************************************************
 * @brief  	灯光状态更新函数
 
 * @param  	
 * @retval 	
 * @author EXcai(亢体造梦)
 *******************************************************************************/
		
	void Update(uint32_t system_time){
		time = system_time;
		if(Command == 1){
			if(Count == 0){Command = 0;last_command_time = time;}
			if(State == 0 && Count != 0 && (time - end_time > 100000)){
				LED_ON();
//				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,50);
				start_time = time;
				State = 1;
			}
			if((time - start_time > 33000) && State == 1 && Count != 0){
				LED_OFF();
//				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
				Count--;
				State = 0;
				end_time = time;
			}
		}
	}
	
		/********************************************************************************
 * @brief  	灯光任务设定函数
 
 * @param  	
 * @retval 	
 * @author EXcai(亢体造梦)
 *******************************************************************************/
	
	void Task(uint32_t system_time,uint8_t count){
		time = system_time;
		if(Command != 1 && (time - last_command_time > 1000000)){
			Command = 1;
			Count = count;
		}
	}
	
	uint8_t Command;
	
#ifndef USE_UPPERMONITOR
	private:
#endif
	uint8_t Count,State;
	uint32_t time,start_time,end_time,last_command_time;
};


