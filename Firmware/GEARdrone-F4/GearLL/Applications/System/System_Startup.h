/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    System_Startup.h
  * @author  EXcai
  * @brief   系统启动文件
  *
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
#pragma once

#ifndef _SYSTEM_STARTUP_H_
#define _SYSTEM_STARTUP_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "i2c_def.h"
#include "GearLL_config.h"
#include "PID.h"
#include "timer_driver.h"
#include "flash_driver.h"
/* Sensors Includes ---------------------------------------------------------*/
#include "SPL06_001.h"
#include "HMC5883L.h"
#include "TOF.h"
#include "MPU.h"
#include "OPTICFLOW.h"
#include "BAT_VOLTAGE.h"
#include "UWB.h"
#include "ESP12F.h"
#include "Serial_Remote.h"

/* Private variables ---------------------------------------------------------*/
extern uint32_t System_time_us;

#ifndef USE_SOFT_IIC
extern I2C_HandleTypeDef hi2c1;
#endif

/* 空闲中断接收数组，数据长度*/
extern uint8_t uart_rxdata[200],data_size;//数传
extern uint8_t optic_flow_data[18],flow_size;//光流
extern uint8_t uwb_data[400];//UWB
extern size_t uwb_size;

#ifdef __cplusplus
extern "C" {
#endif

/* Macro Definitions ---------------------------------------------------------*/
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
	
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8

#define USART1_RX_BUFFER_SIZE 128
#define USART2_RX_BUFFER_SIZE 128
#define USART3_RX_BUFFER_SIZE 128
#define USART4_RX_BUFFER_SIZE 128
#define USART5_RX_BUFFER_SIZE 128

/* HAL Handlers --------------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern void uart6_printf(const char *format,...);

/* Reset Flag Status */
typedef enum
{
	RCC_RESET_FLAG_NONE       = 0x00,       /*!< None Reset Flag    */
	RCC_RESET_FLAG_IWDGRST    = 0x01,       /*!< Independent Watchdog Reset Flag */
	RCC_RESET_FLAG_SFTRST     = 0x02,       /*!< Software Reset Flag    */
	RCC_RESET_FLAG_PORRST     = 0x03,       /*!< POR/PDR Reset Flag    */
	RCC_RESET_FLAG_PINRST     = 0x04,       /*!< PIN Reset Flag     */
	RCC_RESET_FLAG_LPWRRST    = 0x05,       /*!< Low-Power Reset Flag   */
	RCC_RESET_FLAG_WWDGRST    = 0x06,       /*!< Window Watchdog Reset Flag  */
	RCC_RESET_FLAG_BORRST    	= 0x07       	/*!< 欠压复位  */
}RCC_RESET_FLAG_TypeDef;
extern RCC_RESET_FLAG_TypeDef RCC_ResetFlag_GetStatus(void);

/* Classes --------------------------------------------------------------*/
extern SPL06 Spl06;
extern HMC5883L Qmc5883l;
extern VL53L0X Tof;
extern OPTICFLOW Flow;
extern MPU6050 Mpu6050;
extern Voltage_ADC Battery_Voltage;
extern UWB Uwb;
extern ESP12F Esp;

/* RTOS Resources ------------------------------------------------------------*/
/* Task */
void Sensors_Task_200HZ_Function(void *arg);
void Sensors_Task_50HZ_Function(void *arg);
void Process_Task_Function(void *arg);
void Status_Task_Function(void *arg);
void Communication_Task_Function(void *arg);
void Uwb_Task_Function(void *arg);
void NLR_Task_Function(void *arg);
void ESP12F_Task_Function(void *arg);
void Flow_Task_Function(void *arg);



extern TaskHandle_t Sensors_Task_200HZ_Handle;
extern TaskHandle_t Sensors_Task_50HZ_Handle;
extern TaskHandle_t Process_Task_Handle;
extern TaskHandle_t Status_Task_Handle;
extern TaskHandle_t Communication_Task_Handle;
extern TaskHandle_t Uwb_Task_Handle;
extern TaskHandle_t NLR_Task_Handle;
extern TaskHandle_t ESP12F_Task_Handle;
extern TaskHandle_t Flow_Task_Handle;

extern osThreadId defaultTaskHandle;
/* Queue */
extern QueueHandle_t Spl_Port;
extern QueueHandle_t Mag_Port;
extern QueueHandle_t Tof_Port;
extern QueueHandle_t Flow_Port;
extern QueueHandle_t Imu_Port;
extern QueueHandle_t Adc_Port;
extern QueueHandle_t Uart_Port;

/* Semaphore */
extern xSemaphoreHandle uwb_Semaphore;
extern xSemaphoreHandle esp_Semaphore;
extern xSemaphoreHandle flow_Semaphore;

void System_Task_Init(void);
#ifdef __cplusplus
}
#endif

#endif
