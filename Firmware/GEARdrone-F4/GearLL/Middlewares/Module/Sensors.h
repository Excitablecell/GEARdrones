/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Sensors.h
  * @author  EXcai
  * @brief   传感器基类，抽象各种传感器工作流程，方便驱动移植
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
		- 不会请参考HMC5883L:
		- 1.首先新建你的传感器的cpp文件和h文件继承Sensor，确定子类成员变量和成员函数
		- 2.重载Init()、Calibration_Status()、Calibration()、Get_Value()，具体需要做的事情参考Sensors.h内的注释
		- 3.在System_Startup.cpp的文件中新建你的传感器对象
		-	4.在System_Startup.cpp文件中的System_Task_Init函数填写启动传感器函数，例如Qmc5883l.Startup();
		-	5.在Services_Sensors.cpp文件中的Sensors_Task_50HZ_Function或Sensors_Task_200HZ_Function填Run函数，例如Qmc5883l.Run();
		- 6.一切正常的话系统会在执行Calibration()之后，以200HZ或者50HZ执行你的传感器Get_Value()任务
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
**/
#ifndef _SENSORS_H_
#define _SENSORS_H_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
#include <stdint.h>
#include <stddef.h>
#include <limits.h>

#include "main.h"
#include "cmsis_os.h"
#include "filter.h"

/** 
* @brief Class for sensors.
*/
template<typename T>
class Sensor{
	public:
		
//		Sensor(uint32_t _frequency,char* _title,const uint8_t _size,const uint8_t _priority):Task_Frequency(_frequency),Task_title(_title),Stack_Size(_size),Priority(_priority){};
		virtual ~Sensor(){};
			
/************************************************************************************************************************************
 * @brief  	传感器初始化函数
 * @param  	重载Init()以编写传感器初始化
 * @retval 	0: 初始化完成
						1: 初始化失败，自己写个错误处理函数
 * @author	EXcai(亢体造梦)
**************************************************************************************************************************************/	
		virtual uint8_t Init() = 0;
			

/************************************************************************************************************************************
 * @brief  	传感器校准判断函数
 * @param  	重载Calibration_Status()以编写传感器校准判断
 * @retval 	0: 无需校准
						1: 需要校准，系统会自动执行calibration函数，请继承sensor继续重载calibration函数的具体校准流程
 * @author	EXcai(亢体造梦)
**************************************************************************************************************************************/		
		virtual bool Calibration_Status() = 0;
			
			
/************************************************************************************************************************************
 * @brief  	传感器校准函数
 * @param  	重载Calibration()以编写传感器校准流程，重载Calibration()返回1时，Calibration会以10ms为周期被循环执行，直至重载的calibration内返回0！
 * @retval 	0: 校准成功，进行下一步Get_Value()
						1: 正在校准或校准失败，10ms后会重新再运行一次Calibration()校准。
 * @author	EXcai(亢体造梦)
**************************************************************************************************************************************/
		virtual uint8_t Calibration() = 0;
		
/************************************************************************************************************************************
 * @brief  	传感器运行函数
 * @param  	重载Get_Value()以编写传感器运行函数

 * @author	EXcai(亢体造梦)
**************************************************************************************************************************************/	
		virtual T Get_Value() = 0;
	
	
/************************************************************************************************************************************
 * @brief  	传感器执行函数
 
 * @param  	重载Get_value()以编辑传感器运行获取数据的详细步骤，
						重载Calibration_Status()以判断是否需要校准传感器，进入校准流程(注意！需要进行校准则返回1，否则返回0)
						重载Calibration()以编写传感器校准流程，重载Calibration()返回1时，Calibration会以10ms为周期被循环执行，直至重载的calibration内返回0(注意！需要再次校准则返回1，否则返回0)
 * @retval 	0: 成功
						1: 启动失败
 * @author	EXcai(亢体造梦)
**************************************************************************************************************************************/	
			T	Run(){

			/* 判断是否执行校准 */
			T t;
			if(Calibration_Status()){
				while(Calibration()){
					vTaskDelay(10);
					Calibration_state = 1;
				}
				return t;
			}
			else{
				Calibration_state = 0;
				return Get_Value();
			}
		}
		
/**************************************************************************************************************************************
 * @brief  	传感器启动函数
 
 * @param  	重载Init()以初始化传感器，
 
 * @retval 	0: 成功
						1: 启动失败
 * @author	EXcai(亢体造梦)
**************************************************************************************************************************************/	
		uint8_t Startup(){
			return Init();
		}
		
//		static void staticTask_Function(void *pvParameter);
		void Task_Function();
		TaskHandle_t Task_Handle;

		uint8_t Calibration_state;
	protected:
		uint32_t Task_Frequency;
};

#endif

#endif
