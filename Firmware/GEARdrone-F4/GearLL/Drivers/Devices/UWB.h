/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    UWB.h
  * @author  EXcai
  * @brief   LINKTRACK UWB驱动，包含数传数据包(nodeframe0)解包，LP模式(nodeframe2)解包，DR模式(nodeframe3)解包
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

#ifndef __UWB_H_
#define __UWB_H_
	
/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include <string.h>
#include "Sensors.h"
#include "math.h"
	
#include "nlink_linktrack_nodeframe0.h"// data receive
#include "nlink_linktrack_nodeframe2.h"// LP mode need to use frame2
#include "nlink_linktrack_nodeframe3.h"// DR mpde need to use frame3
#include "nlink_utils.h"

/* Variables -------------------------------------------------------------------*/

extern UART_HandleTypeDef huart2;
extern uint8_t uwb_data[400];
extern size_t uwb_size;

extern nlt_nodeframe0_t g_nlt_nodeframe0;
extern nlt_nodeframe2_t g_nlt_nodeframe2;
extern nlt_nodeframe3_t g_nlt_nodeframe3;

/** 
* @brief 无人机间传输的位置和速度,以及其他无人机间距离
*/

struct uwb_trans_data{
	uint8_t data[68];
	uint16_t data_length;
};

/** 
* @brief 传感器结构体
*/

struct uwb_struct_data{
	
	uint8_t State;
	uint8_t Mode;
	uint8_t role;
  uint8_t id;
	uint32_t system_time;
	uint8_t valid_node_count;
	
	struct uwb_trans_data others_data[5];
	
	float pos_3d[3];
  float eop_3d[3];//精度
  float vel_3d[3];
	float distanse[5];
};

/** 
* @brief Class for uwb.
*/
class UWB : public Sensor<uwb_struct_data>{
public:
	UWB(){};
	virtual ~UWB(){};
	
	void User_Unpack(void){
		if(uwb_data[1] == 0x04){
			if(g_nlt_nodeframe2.UnpackData(uwb_data,uwb_size)){//LP局部定位模式解包（解出pos vel eop）
				Mode = LP_Mode;
				role = g_nlt_nodeframe2.result.role;
				id = g_nlt_nodeframe2.result.id;
				system_time = g_nlt_nodeframe2.result.system_time;
				valid_node_count = g_nlt_nodeframe2.result.valid_node_count;
				
				for(int i=0;i<3;i++){
					pos_3d[i] = g_nlt_nodeframe2.result.pos_3d[i];
					eop_3d[i] = g_nlt_nodeframe2.result.eop_3d[i];
					vel_3d[i] = g_nlt_nodeframe2.result.vel_3d[i];
				}
			}
		}
		else if(uwb_data[1] == 0x05){
			if(g_nlt_nodeframe3.UnpackData(uwb_data,uwb_size)){//DR分布式测距模式解包（解出distance）
				Mode = DR_Mode;
				role = g_nlt_nodeframe3.result.role;
				id = g_nlt_nodeframe3.result.id;
				system_time = g_nlt_nodeframe3.result.system_time;
				valid_node_count = g_nlt_nodeframe3.result.valid_node_count;
				
				for(int i=0;i<valid_node_count;i++){
					uint8_t id;
					id = g_nlt_nodeframe3.result.nodes[i]->id;
					distanse[id] = g_nlt_nodeframe3.result.nodes[i]->dis;
				}
			}
		}
		else if(uwb_data[1] == 0x02){
			if(g_nlt_nodeframe0.UnpackData(uwb_data,uwb_size)){//获取它机的public数据
				role = g_nlt_nodeframe0.result.role;
				id = g_nlt_nodeframe0.result.id;
				valid_node_count = g_nlt_nodeframe0.result.valid_node_count;
				
				for(int i=0;i<valid_node_count;i++){//获取它机测量的所有distance，但不解包，在drone的get sensor data中解包
					uint8_t id;
					id = g_nlt_nodeframe0.result.nodes[i]->id;
					others_data[id].data_length = g_nlt_nodeframe0.result.nodes[i]->data_length;
		//				for(int j=0;j<116;i++){others_data[id].data[j] = g_nlt_nodeframe0.result.nodes[i]->data[j];}
					memcpy(&others_data[id].data,&g_nlt_nodeframe0.result.nodes[i]->data,68);
		//				memset(g_nlt_nodeframe0.result.nodes[i]->data,0,68);
				}
			}
		}
//		memset(uwb_data,0,400);
//		uwb_size = 0;
		HAL_UART_Receive_DMA(&huart2, uwb_data, 400);
	}
		
	
	virtual uint8_t Init(){
		
		User_Unpack();
		
		return 0;
	}
	
	virtual uint8_t Calibration(){return 0;}
	virtual bool Calibration_Status(){return 0;}
	virtual struct uwb_struct_data Get_Value(){
		
		struct uwb_struct_data data;
		
		User_Unpack();
		
		data.Mode = Mode;
		data.role = role;
		data.id = id;
		data.system_time = system_time;
		data.valid_node_count = valid_node_count;
		if(Mode == LP_Mode){
			for(int i=0;i<3;i++){
				data.pos_3d[i] = pos_3d[i];
				data.eop_3d[i] = eop_3d[i];
				data.vel_3d[i] = vel_3d[i];
			}
		}
		else if(Mode == DR_Mode){
			for(int i=0;i<5;i++){data.distanse[i] = distanse[i];};
		}
		
		for(int i=0;i<5;i++){data.others_data[i] = others_data[i];};
		
		return data;
	}
	uint8_t Status;
	uint8_t Mode;
	
	uint8_t role;
  uint8_t id;
	uint32_t system_time;
	uint8_t valid_node_count;
	
	uwb_trans_data others_data[9];
	float pos_3d[3];
  float eop_3d[3];//精度
  float vel_3d[3];
	float distanse[9];
	
	#ifndef USE_UPPERMONITOR
	private:
	#endif
	
};


#endif
#ifdef __cplusplus
}
#endif
