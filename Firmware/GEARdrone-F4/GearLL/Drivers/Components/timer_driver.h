/**
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  ******************************************************************************
  */
#ifndef  _TIMER_DRIVER_H
#define  _TIMER_DRIVER_H

#ifdef  __cplusplus
extern "C"{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Private macros ------------------------------------------------------------*/
#define microsecond()    Get_SystemTimer()

/* Private type --------------------------------------------------------------*/
typedef struct{
  uint32_t last_time;	/*!< Last recorded real time from systick*/
  float dt;				/*!< Differentiation of real time*/
}TimeStamp;

/* Exported macros -----------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void Timer_Init(TIM_HandleTypeDef* htim);
void Update_SystemTick(void);
uint32_t Get_SystemTimer(void);
void delay_ms_nos(uint32_t cnt);
void delay_us_nos(uint32_t cnt);


#ifdef  __cplusplus
}
#endif

#endif 
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
