/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    System_Startup.cpp
  * @author  EXcai
  * @brief   系统启动文件
  *
  ==============================================================================
													How to use this library 
  ==============================================================================
    @note
			- 系统启动文件

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
#include "System_Startup.h"

uint32_t System_time_us;
VL53L0X_Error Status=VL53L0X_ERROR_NONE;//工作状态

QueueHandle_t Spl_Port;
QueueHandle_t Mag_Port;
QueueHandle_t Tof_Port;
QueueHandle_t Flow_Port;
QueueHandle_t Imu_Port;
QueueHandle_t Adc_Port;
QueueHandle_t Uart_Port;

xSemaphoreHandle uwb_Semaphore = NULL;
xSemaphoreHandle esp_Semaphore = NULL;
xSemaphoreHandle flow_Semaphore = NULL;

/********************************************************************************
 * @brief  	单片机异常复位查询
 
 * @param  	查询单片机异常复位的原因
 * @retval 	
 * @author EXcai(亢体造梦)从网上抠的
 *******************************************************************************/
RCC_RESET_FLAG_TypeDef RCC_ResetFlag_GetStatus(void)
{
 RCC_RESET_FLAG_TypeDef ResetStatusFlag = RCC_RESET_FLAG_NONE;
 
 if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) 
 { 
  ResetStatusFlag = RCC_RESET_FLAG_IWDGRST;
 }
 else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) 
 {
  ResetStatusFlag = RCC_RESET_FLAG_SFTRST;
 }
 else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) 
 {
  ResetStatusFlag = RCC_RESET_FLAG_PORRST;
 }
 else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) 
 {
  ResetStatusFlag = RCC_RESET_FLAG_PINRST;
 }
 else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) 
 {
  ResetStatusFlag = RCC_RESET_FLAG_LPWRRST;
 }
 else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
 {
  ResetStatusFlag = RCC_RESET_FLAG_WWDGRST;
 }
 else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
 {
  ResetStatusFlag = RCC_RESET_FLAG_BORRST;
 }

 __HAL_RCC_CLEAR_RESET_FLAGS();
 
 return ResetStatusFlag;
}

/********************************************************************************
 * @brief  	application freertos init function.
 
 * @param  	在这放置传感器startup函数，创建传感器周期读取freertos任务，编辑各个传感器初始化顺序
 * @retval 	
 * @author EXcai(亢体造梦)
 *******************************************************************************/
void System_Task_Init(void)
{
	
	vSemaphoreCreateBinary(uwb_Semaphore);
	vSemaphoreCreateBinary(esp_Semaphore);
	vSemaphoreCreateBinary(flow_Semaphore);
	
  /* Queue Init *///队列会造成时延，未使用
//	Uart_Port = xQueueCreate(2, sizeof(uart_rxdata));
//  Spl_Port = xQueueCreate(1, sizeof(pressure_data));
//	Adc_Port = xQueueCreate(1, sizeof(adc_data));
//	Tof_Port = xQueueCreate(1, sizeof(tof_data));
//	Flow_Port = xQueueCreate(1, sizeof(flow_data));
//	Mag_Port = xQueueCreate(1, sizeof(mag_data));
//	Imu_Port = xQueueCreate(1, sizeof(imu_data));
	
  /* HAL Init */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//蜂鸣器
	
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	
	
	HAL_UART_Receive_DMA(&huart1, optic_flow_data,18);
	HAL_UART_Receive_DMA(&huart2, uwb_data, 400);
	
	HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);//关蜂鸣器
	
  /* Task Init */
	Timer_Init(&htim5);
	myPIDTimer::getMicroTick_regist(Get_SystemTimer);
	xTaskCreate(Sensors_Task_200HZ_Function, 	"200HZ_Sensors_Task",   Normal_Stack_Size, NULL, PriorityNormal, &Sensors_Task_200HZ_Handle);
	xTaskCreate(Sensors_Task_50HZ_Function, 	"50HZ_Sensors_Task",   	Large_Stack_Size, NULL, PriorityNormal, &Sensors_Task_50HZ_Handle);
	xTaskCreate(Process_Task_Function, 	"Process_Task",   	Normal_Stack_Size, NULL, PriorityNormal, &Process_Task_Handle);
	xTaskCreate(Status_Task_Function, 	"Status_Machine_Task",   	Normal_Stack_Size, NULL, PriorityNormal, &Status_Task_Handle);
	xTaskCreate(Communication_Task_Function, 	"Communication_Task",   	Normal_Stack_Size, NULL, PriorityBelowNormal, &Communication_Task_Handle);
	xTaskCreate(Uwb_Task_Function, 	"Uwb_unpack_Task",   	Huge_Stack_Size, NULL, PriorityNormal, &Uwb_Task_Handle);
	xTaskCreate(NLR_Task_Function, 	"NLR_estimate_Task",   	Large_Stack_Size, NULL, PriorityNormal, &NLR_Task_Handle);
	xTaskCreate(ESP12F_Task_Function, 	"ESP12F_Task",   	Normal_Stack_Size, NULL, PriorityNormal, &ESP12F_Task_Handle);
	xTaskCreate(Flow_Task_Function, 	"Flow_unpack_Task",   	Small_Stack_Size, NULL, PriorityAboveNormal, &Flow_Task_Handle);
	
	/* Sensors Init */
	LED_ON();
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,50);
	Spl06.Startup();HAL_Delay(5);
	Qmc5883l.Startup();HAL_Delay(5);
	
	HAL_GPIO_WritePin(XSHUT_GPIO_Port,XSHUT_Pin,GPIO_PIN_RESET);HAL_Delay(50);//vl53l0复位
	HAL_GPIO_WritePin(XSHUT_GPIO_Port,XSHUT_Pin,GPIO_PIN_SET);HAL_Delay(50);//vl53l0复位
	
	Tof.Startup();HAL_Delay(5);
	Flow.Startup();HAL_Delay(5);
	Battery_Voltage.Startup();HAL_Delay(5);
	Mpu6050.Startup();HAL_Delay(5);
	Uwb.Startup();HAL_Delay(5);
	Esp.Startup();HAL_Delay(5);

	LED_OFF();//LED off
	
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
	/* 你可以在下面添加你的传感器启动函数 */
	//例如：Xxx.Startup();HAL_Delay(5);
	
}

