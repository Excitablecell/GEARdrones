/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    ESP12F.h
  * @author  EXcai
  * @brief   ESP8266 MQTT固件驱动
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

#ifndef __ESP12F_H_
#define __ESP12F_H_
	
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Sensors.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Variables -------------------------------------------------------------------*/

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern uint8_t uart_rxdata[200],data_size;

extern void uart6_printf(const char *format,...);
	
extern uint8_t STATE[60];
extern uint8_t RST[60];
extern uint8_t WIFI_MODE_SET[60];
extern uint8_t WIFI_CONFIG[40];
extern uint8_t MQTT_CONFIG[60];
extern uint8_t MQTT_CONNECT[60];

extern uint8_t MQTT_SEND[60];
extern uint8_t MQTT_SUB[60];
extern uint8_t MQTT_REMOTE[60];

/** 
* @brief ESP12F数据
*/

struct uart_data{
	uint8_t Status;
	uint32_t timestamp_ms;
	uint8_t Remote_Mode;
	uint32_t Remote_time;
	
	float pos_x_target,pos_y_target,pos_z_target;
	float speed_x_target,speed_y_target,speed_z_target;
	float pitch_target,roll_target,yaw_target;
};

#ifdef __cplusplus
}
#endif

/** 
* @brief Class for esp12f.
*/
class ESP12F : public Sensor<uint8_t>{
public:
	ESP12F(){};
	virtual ~ESP12F(){};
		
	virtual uint8_t Init(){
		printf("AT+RST\r\n");
		return 0;
	}
	
	virtual uint8_t Calibration(){return 0;}
	virtual bool Calibration_Status(){return 0;}
	virtual uint8_t Get_Value(){
		net_delay = System_time_us - heart;
		char* ret;
		
		ret = strchr((char*)uart_rxdata,(int)'[');
		if(ret != NULL && ret[5] == ']' && ret[0] == '['){
			
			if(Mode != 4 && Mode != 2){Mode = 1;}
			heart = System_time_us;
			
			if(abs(ret[1] - '0' -5) < 6){target_roll = (ret[1] - '0' -5)*4;}else{target_roll = 0;}
			if(abs(ret[2] - '0' -5) < 6){target_pitch = -(ret[2] - '0' -5)*4;}else{target_pitch = 0;}
			if(abs(ret[3] - '0' -5) < 6){target_height = (ret[3] - '0');}
			target_yaw = 0;
			
			if(target_height > target_height_max && Mode != 4){target_height_max = target_height;Mode = 2;}//表示起飞指令下达过
			if(target_height < 1 && target_height_max > 0){target_height = 0;Mode = 4;}//降落指令：起飞指令下达过后且摇杆量归零
		}
		if(System_time_us - heart > 900000){Mode = 0;}//遥控丢失 900ms
		memset(uart_rxdata,0,200);
		return Mode;
	}
	
	uint8_t ESP12F_Send(float data0,float data1,float data2,uint8_t system_id,uint32_t system_time,uint8_t type){
		// (\\\")表示在接收端为:(") \\,表示在接收端为:逗号
		if(type == 1){
			printf("AT+MQTTPUB=0,\"MAV%u_TX\",\"{\\\"type\\\":1\\,\\\"time\\\":%u\\,\\\"value1\\\":%f\\,\\\"value2\\\":%f\\,\\\"value3\\\":%f}\",1,0\r\n",system_id,system_time,data0,data1,data2);
		}
		return 0;
	}
	
	uint8_t State;
	uint8_t Init_State;
	uint32_t heart,net_delay;
	float target_roll,target_pitch,target_yaw,target_height,target_height_max;
	
#ifndef USE_UPPERMONITOR
	private:
#endif
	uint8_t Mode;
	uint8_t Step_passed[6];
};
#endif

