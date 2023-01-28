/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    GearLL_config.h
  * @author  EXcai(亢体造梦)
  * @brief   GearLL库配置文件，用于在软硬件不同方案间切换
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
#pragma once

#ifndef __GEARLL_CONFIG_H__
#define __GEARLL_CONFIG_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include <string.h>

#define LP_Mode 0 //uwb局部定位模式
#define DR_Mode 1 //uwb测距模式

#define _CONTROL_START_TIME 10000 //开始飞行时间
#define _CONTROL_LIMIT_TIME 18000 //限制飞行时间（只在调试模式生效

#define _LIMIT_HEIGHT 1.4f //限制飞行高度
#define _TARGET_HEIGHT 0.4f //目标飞行高度

#define LED_ON() HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);//LED 点亮
#define LED_OFF() HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);//LED off

extern uint32_t System_time_us;

/*********************************************************************************
	* @brief  磁力计使用HMC5883L的国产替代:QMC5883L
	* @param  -注释该语句,启用该define,即为使用HMC5883L的驱动，反之则为QMC5883L
						-使用QMC5883L（国产替代版HMC5883L），寄存器位置和HMC不一致(实际上垃圾得一批，数据又抖又歪,狗都不用)

	* @author EXcai亢体造梦
	********************************************************************************/

#define USE_QMC 

/*********************************************************************************
	* @brief  对MPU使用软件iic
	* @param  该语句为使用软件IIC的驱动MPU6050，好处是不会因为stm32的硬件iic bug而造成无人机不能正常启动

	* @author EXcai亢体造梦
	********************************************************************************/

//#define USE_SOFT_IIC

/*********************************************************************************
	* @brief  使用调试上位机
	* @param  -使用调试上位机
						-此模式下所有private和protected成员都会变成public 便于用uppermonitor查看

	* @author EXcai亢体造梦
	********************************************************************************/
	
//#define USE_UPPERMONITOR 

#ifdef USE_UPPERMONITOR
	#include "UpperMonitor.h"
#endif

/*********************************************************************************
	* @brief  使用调试调试模式
	* @param  -该模式下无人机启动后自动进入姿态模式，跳过起飞状态

	* @author EXcai亢体造梦
	********************************************************************************/

//#define DEBUG_MODE

/*********************************************************************************
	* @brief  不要转动螺旋桨
	* @param  取消注释,启用该define,无人机将不会转动螺旋桨！

	* @author EXcai亢体造梦
	********************************************************************************/

//#define NO_MOTOR

#endif
