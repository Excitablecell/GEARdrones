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
#ifndef __UWB_H_
#define __UWB_H_
	
#define LP_Mode 0 //uwb局部定位模式
#define DR_Mode 1 //uwb测距模式

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include <string.h>
#include "math.h"
#include "cmsis_os.h"
	
#include "nlink_linktrack_nodeframe0.h"// data receive
#include "nlink_linktrack_nodeframe2.h"// LP mode need to use frame2
#include "nlink_linktrack_nodeframe3.h"// DR mpde need to use frame3
#include "nlink_utils.h"
#include "Butterworth.h"

/* Variables -------------------------------------------------------------------*/

extern uint8_t uwb_data[200];
extern size_t uwb_size;

extern nlt_nodeframe0_t g_nlt_nodeframe0;
extern nlt_nodeframe2_t g_nlt_nodeframe2;
extern nlt_nodeframe3_t g_nlt_nodeframe3;
extern ButterworthFilter BWF[3];

/** 
* @brief 无人机间传输的位置和速度,以及其他无人机间距离
*/

struct uwb_trans_data{
	linktrack_role_e other_role;
	uint8_t id;
	uint8_t data[52];
	uint16_t data_length;
};

struct uwb_position{
	uint8_t id;
	float pos[3];
};

/** 
* @brief 传感器结构体
*/

/** 
* @brief Class for uwb.
*/
class UWB{
public:
	UWB(){};
	~UWB(){};
	
	void User_Unpack(void){
		if(uwb_data[1] == 0x04){//LP局部定位模式解包（解出pos vel eop）
			anchor_count = 0;
			if(g_nlt_nodeframe2.UnpackData(uwb_data,uwb_size)){
				Mode = LP_Mode;
				role = g_nlt_nodeframe2.result.role;
				id = g_nlt_nodeframe2.result.id;
				network_time = g_nlt_nodeframe2.result.system_time;
				valid_node_count = g_nlt_nodeframe2.result.valid_node_count;
				
				for(int i=0;i<3;i++){
					last_pos_3d[i] = pos_3d[i];
					pos_3d[i] = BWF[i].f(g_nlt_nodeframe2.result.pos_3d[i]);
					vel_3d_estimate[i] = (pos_3d[i] - last_pos_3d[i]);
					eop_3d[i] = g_nlt_nodeframe2.result.eop_3d[i];
					vel_3d[i] = g_nlt_nodeframe2.result.vel_3d[i];
				}
				
				/*统计在线的anchor数量*/
				for(int i=0;i<valid_node_count;i++){
					if(g_nlt_nodeframe2.result.nodes[i]->role == 1){//anchor
						anchor_count++;
					} 
				}
			}
		}
		else if(uwb_data[1] == 0x02){//数传模式解包
			if(g_nlt_nodeframe0.UnpackData(uwb_data,uwb_size)){
				
				//本机数据
				role = g_nlt_nodeframe0.result.role;
				id = g_nlt_nodeframe0.result.id;
				valid_node_count = g_nlt_nodeframe0.result.valid_node_count;
				
				for(int i=0;i<valid_node_count;i++){//获取所有它机数传

					if(g_nlt_nodeframe0.result.nodes[i]->role == 3){//遥控器
						remote_status = 1;
						remote_data.other_role = g_nlt_nodeframe0.result.nodes[i]->role;
						remote_data.data_length = g_nlt_nodeframe0.result.nodes[i]->data_length;
						memcpy(&remote_data.data,&g_nlt_nodeframe0.result.nodes[i]->data,remote_data.data_length);
//						memcpy(&drones_pos[0].pos,&remote_data.data[4],12);drones_pos[0].id = 1;
//						memcpy(&drones_pos[1].pos,&remote_data.data[16],12);drones_pos[1].id = 2;
//						memcpy(&drones_pos[2].pos,&remote_data.data[28],12);drones_pos[2].id = 3;
//						memcpy(&drones_pos[3].pos,&remote_data.data[40],12);drones_pos[3].id = 4;
					}
					else if(g_nlt_nodeframe0.result.nodes[i]->role == 2){//anchor
						computer_command.other_role = g_nlt_nodeframe0.result.nodes[i]->role;
						computer_command.data_length = g_nlt_nodeframe0.result.nodes[i]->data_length;
						memcpy(&computer_command.data,&g_nlt_nodeframe0.result.nodes[i]->data,remote_data.data_length);
					}
				}
			}
		}
	}
		
	uint8_t Status;
	uint8_t Mode;
	
	uint8_t role;
  uint8_t id;
	uint32_t system_time,network_time;
	uint8_t valid_node_count,remote_status,anchor_count;
	
	uwb_trans_data remote_data,computer_command;
	uwb_position drones_pos[4];
	float time_span;
	float pos_3d[3],last_pos_3d[3];//本次、上次位置
  float eop_3d[3];//精度
  float vel_3d[3],vel_3d_estimate[3];//UWB模块回传速度、位置差分估计的速度
	
};


#endif
