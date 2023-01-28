/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Serial_Remote.h
  * @author  EXcai
  * @brief   串口遥控器
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


#ifndef __SERIAL_REMOTE_H_
#define __SERIAL_REMOTE_H_

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "Sensors.h"
#include "filter.h"
#include "arm_math.h"
#include "timer_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Variables -------------------------------------------------------------------*/
	
struct serial_data{
	
	uint8_t Status;
	uint32_t timestamp_ms;
	
	float x_target,y_target,z_target,basic_pwm;
};
	
extern uint8_t uart_rxdata[200],data_size;//数传
extern UART_HandleTypeDef huart6;

#ifdef __cplusplus
}
#endif

/** 
* @brief Class for Serial_Remote.
*/
class Serial_Remote{
public:
	Serial_Remote(){};
 ~Serial_Remote(){};

	uint8_t Get_Remote(serial_data *remote){
		uint32_t timestamp;
		if(uart_rxdata[0] == 0x11 && uart_rxdata[7] == 0x22){
			
			memcpy(rx_watch1_stack,uart_rxdata,8);//debug的时候观察遥控器数据
			
			heart = Get_SystemTimer()/1000;
			uint8_t takeoff_flag;
			takeoff_flag = uart_rxdata[3];
			
			if(takeoff_flag == 0x00){Status = 1;}
			
			uint8_t Check_sum=(uint8_t)(uart_rxdata[1]+uart_rxdata[2]+uart_rxdata[4]+uart_rxdata[5]);
			if(takeoff_flag == 0xDD && Check_sum == uart_rxdata[6]){
				
				remote->y_target = (uart_rxdata[1] - 50.0f);
				remote->x_target = (uart_rxdata[2] - 50.0f);
				
				if(remote->z_target > 360.0f){remote->z_target = (remote->z_target + (uart_rxdata[5] - 50.0f)/40.0f) - 360.0f;}
				if(remote->z_target < -360.0f){remote->z_target = (remote->z_target + (uart_rxdata[5] - 50.0f)/40.0f) + 360.0f;}
				else{remote->z_target = (remote->z_target + (uart_rxdata[5] - 50.0f)/40.0f);}
				
				remote->basic_pwm = (50 - uart_rxdata[4])*15.0f;
				if(uart_rxdata[4] > 0x65){
					Status = 0;//紧急停机
				}
				else{
					Status = 2;
				}
			}
			else{//如果校验和不满足
				remote->y_target = 0;
				remote->x_target = 0;
			}
			memset(uart_rxdata,0,200);
		}
		else{//如果没收到数据
			timestamp = Get_SystemTimer()/1000;
			if(timestamp - heart > 600){
				Status = 0;
			}//超过600ms认为掉线
		}
		
		return Status;
	}
	 
	uint32_t heart;
#ifndef USE_UPPERMONITOR
	private:
#endif
	uint8_t Status;//0:offline 1: online 2:allow take-off 3:formation mode
	uint8_t rx_watch1_stack[8];
};

#endif
