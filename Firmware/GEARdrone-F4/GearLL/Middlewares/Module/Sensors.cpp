/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Sensors.cpp
  * @author  EXcai
  * @brief   传感器基类，抽象各种传感器工作流程，方便驱动移植
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
		- 参考HMC5883L:
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
  */
/* Includes ------------------------------------------------------------------*/
#include "Sensors.h"

/* function prototypes -------------------------------------------------------*/

/**
 * @brief  	为了调用类成员函数，需要一个实例指针,需要这种间接方式将成员函数作为回调传递给 C 系统
 
 * @retval 	
 * @author EXcai(亢体造梦)
 */
//void Sensor::staticTask_Function(void *pvParameter)
//{
//	Sensor* sensor = reinterpret_cast<Sensor*>(pvParameter); //obtain the instance pointer
//	sensor->Task_Function(); //dispatch to the member function, now that we have an instance pointer
//}
