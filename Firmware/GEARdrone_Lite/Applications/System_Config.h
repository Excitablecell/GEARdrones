/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    System_Config.h
  * @author  EXcai(亢体造梦)
  * @brief   配置文件，用于在软硬件不同方案间切换
  *
  *******************************************************************************/
#pragma once

#ifndef __SYSTEM_CONFIG_H__
#define __SYSTEM_CONFIG_H__

/*********************************************************************************
	* @brief  电机停转(用于测试各个功能是否正常)
	* @param  -1:电机不要转动
						-0:电机可以转动

	* @author EXcai亢体造梦
	********************************************************************************/
#define DISABLE_MOTORS 0

/*********************************************************************************
	* @brief  使用UWB定位，确保UWB基站已部署并标定！
	* @param  -1:使用UWB提供位置，速度
						-0:使用光流传感器提供位置，速度

	* @author EXcai亢体造梦
	********************************************************************************/
#define USE_UWB 0

#define LP_Mode 0 //uwb局部定位模式
#define DR_Mode 1 //uwb测距模式

/*********************************************************************************
	* @brief  使用磁力计
	* @param  -1:使用磁力计
						-0:不使用磁力计
						
	* @note		在目前版本的PCB上，电机一转这玩意的数值就会飞。

	* @author EXcai亢体造梦
	********************************************************************************/
#define USE_MAGSENSOR 0

/*********************************************************************************
	* @brief  使用气压计
	* @param  -1:使用气压计
						-0:不使用气压计
						
	* @note		气压计要做好海绵包裹！！防止桨叶气流扰动影响采样！！！

	* @author EXcai亢体造梦
	********************************************************************************/
#define USE_BAROMETER 0 

/*********************************************************************************
	* @brief  使用matlab进行远程二阶控制
	* @param  -1:使用matlab二阶控制

	* @author EXcai亢体造梦
	********************************************************************************/
#define MATLAB_SECOND_OREDER_CONTROL 0

/*********************************************************************************
	* @brief  切换控制器
	* @param  -1:输出调节位置控制(未完成)
						-0:PID位置控制

	* @author EXcai亢体造梦
	********************************************************************************/
#define OUTPUT_REGULATION_CONTROLLER 0

/*********************************************************************************
	* @brief  使能机内绕圆
	* @param  -1:生成并使用机内绕圆轨迹

	* @author EXcai亢体造梦
	********************************************************************************/
#define RUN_CIRCLE_PATH 0
#define RADIUS 2.0 //半径
#define T 0.01 //周期
#define WR 0.4 //角速度

#define CENTER_X_POS -5 //圆心x位置
#define CENTER_Y_POS 5 //圆心y位置

/*********************************************************************************
	* @brief  使能缓慢上升
	* @param  -1:使无人机从0.4m向更高高度缓慢爬升

	* @author EXcai亢体造梦
	********************************************************************************/
#define DYNAMIC_HEIGHT 1

#define HEIGHT_TARGET 0.6f

/*********************************************************************************
* @brief  切换传输数据的网络协议
	* @param  -0:使用UDP传输数据(对应远端GEARmonitor_UDP.py)
						-1:使用MQTT协议传输数据(对应远端GEARmonitor_MQTT.py runn.m monitor2.m)

	* @author EXcai亢体造梦
	********************************************************************************/
#define NETWORK_PROTOCOL 1
#define MQTT_TEMPLATE "AT+MQTTPUB=0,\"MAV%u_TX\",\"{\\\"type\\\":1\\,\\\"time\\\":%u\\,\\\"value1\\\":%.3f\\,\\\"value2\\\":%.3f\\,\\\"value3\\\":%.3f}\",1,0\r\n"
#define MQTT_TEMPLATE_FLOAT "AT+MQTTPUB=0,\"MAV%u_TX\",\"{\\\"type\\\":1\\,\\\"time\\\":%.3f\\,\\\"value1\\\":%.3f\\,\\\"value2\\\":%.3f\\,\\\"value3\\\":%.3f}\",1,0\r\n"

/*********************************************************************************
* @brief  发送格式
	* @param  -0:GEARmonitor
						-1:matlab 单机
						-2:matlab 多机
						-3:uwb 多机机间通信

	* @author EXcai亢体造梦
	********************************************************************************/
#define SEND_TYPE 5

/*********************************************************************************
* @brief  切换红外测距芯片方案
	* @param  -0:VL53L0X
						-1:VL53L1

	* @author EXcai亢体造梦
	********************************************************************************/
#define TOF_SENSOR 1

#endif
