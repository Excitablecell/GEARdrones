/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    OPTICFLOW.h
  * @author  EXcai
  * @brief   光流驱动
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
#ifdef __cplusplus
extern "C" {
#endif

#ifndef __OPTICFLOW_H_
#define __OPTICFLOW_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include <string.h>
#include "Sensors.h"
#include "math.h"

/* Variables -------------------------------------------------------------------*/

extern uint8_t optic_flow_data[18],flow_size;
extern uint32_t System_time_us;
	
struct flow_data{
	
	uint8_t Status;
	uint32_t timestamp_ms,time_span_ms;
	
	int16_t flow_x,flow_y;	//光流数据
	float sum_flow_x, sum_flow_y;		//光流数据积分，偏差位置
};
	
extern uint8_t OPTICFLOW_Init(void);
extern UART_HandleTypeDef huart1;

/** 
* @brief Class for OPTICFLOW.
*/
class OPTICFLOW : public Sensor<flow_data>{
public:
	OPTICFLOW(){};
	virtual ~OPTICFLOW(){};

	virtual uint8_t Init(){
		memcpy(optic_flow,optic_flow_data,9);
		return 0;
	}
	
	virtual uint8_t Calibration(){return 0;}
	virtual bool Calibration_Status(){return 0;}
	virtual struct flow_data Get_Value(){
		
		flow_data data;
		
		memcpy(optic_flow,optic_flow_data,9);
		if(optic_flow[0] == 0xFE)
		{
			uint8_t Check_sum=(uint8_t)(optic_flow[2]+optic_flow[3]+optic_flow[4]+optic_flow[5]);
			if(Check_sum == optic_flow[6]){
				data.Status = 0;
				capture_time[1] = capture_time[0];
				capture_time[0] = System_time_us/1000;
				time_span_ms = capture_time[0] - capture_time[1];
				
				flow_y = (int16_t)(optic_flow[3]<<8)|optic_flow[2];//roll 右移+
				flow_x = (int16_t)(optic_flow[5]<<8)|optic_flow[4];
				
//				//积分环节
//				if(System_time_us < 8000000){//8秒内不进行光流位置计算，不准确
//					sum_flow_x += 0.033f * flow_x;
//					sum_flow_y += 0.033f * flow_y;
//				}
				
			}
			else{
				data.Status = 1;
			}
		}
		
		data.time_span_ms = time_span_ms;
		data.timestamp_ms = capture_time[0];
		data.flow_x = flow_x;
		data.flow_y = flow_y;
//		data.sum_flow_x = sum_flow_x;
//		data.sum_flow_y = sum_flow_y;
		
		return data;
	}
	uint8_t Status;
	int16_t flow_x,flow_y;	//光流数据
	double sum_flow_x, sum_flow_y;		//光流数据积分，偏差位置
	uint32_t capture_time[2],time_span_ms;
	
	#ifndef USE_UPPERMONITOR
	private:
	#endif
	uint8_t optic_flow[9];
};


#endif

#ifdef __cplusplus
}
#endif
