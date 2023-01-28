/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Services_Communication.cpp
  * @author  EXcai
  * @brief   无人机工作服务
  *
  ==============================================================================
													How to use this library 
  ==============================================================================
    @note
			- 无人机通讯服务

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
#include "Drone.h"

/* Private variables ---------------------------------------------------------*/
extern GearDrone Drone;

extern pressure_data 	Spl_temp;
extern mag_data 			Mag_temp;
extern tof_data 			Tof_temp;
extern flow_data 			Flow_temp;
extern imu_data 			Imu_temp;
extern adc_data 			Adc_temp;
extern uwb_struct_data				Uwb_temp;
extern uart_data					Uart_temp;

/* Tasks Handle variables ----------------------------------------------------*/
TaskHandle_t Communication_Task_Handle;
TaskHandle_t ESP12F_Task_Handle;

/********************************************************************************
 * @brief  数据通信任务
 
 * @param  	
 * @retval 	
 * @author EXcai(亢体造梦)
 *******************************************************************************/

void Communication_Task_Function(void *arg){
	
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	
	for(;;){
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t,60);
		System_time_us = Get_SystemTimer();
		if(System_time_us > 3000000 && Esp.Init_State != 2){//启动ESP12F
//			Drone.Remote_Mode = Esp.Run();
			vTaskDelay(200);
			printf("AT+CWMODE=1\r\n");
			vTaskDelay(500);
			printf("AT+CWJAP=\"%s\",\"%s\"\r\n","GU","2808278066");
			vTaskDelay(2000);
			printf("AT+MQTTUSERCFG=0,1,\"MAV%u\",\"%s\",\"%s\",0,0,\"\"\r\n",Drone.Flash_ID,"admin","public");
			vTaskDelay(500);
			printf("AT+MQTTCONN=0,\"%s\",1883,0\r\n","192.168.0.139");
			vTaskDelay(1000);
			printf("AT+MQTTSUB=0,\"MAV%u_RX\",0\r\n",Drone.Flash_ID);
			vTaskDelay(500);
			printf("AT+MQTTSUB=0,\"%s\",0\r\n","Remote");
			vTaskDelay(500);
			memset(uart_rxdata,0,200);
			Esp.Init_State = 2;
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
			HAL_UART_Receive_DMA(&huart6, uart_rxdata, 200);
//			vTaskSuspend(ESP_Init_Task_Handle);
		}
		Drone.Communication(&Uwb_temp);
	}
}

/********************************************************************************
 * @brief  ESP12F通信任务
 
 * @param  	
 * @retval 	
 * @author EXcai(亢体造梦)
 *******************************************************************************/

void ESP12F_Task_Function(void *arg){
	
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	
	for(;;){
		/* wait for next circle */
//		vTaskDelayUntil(&xLastWakeTime_t,20);
		if(xSemaphoreTake(esp_Semaphore, portMAX_DELAY) == pdTRUE && Esp.Init_State == 2 && Drone.Flash_ID == 0){//ESP12F接收中断触发
			Drone.Remote_Mode = Esp.Get_Value();
			if(Drone.Remote_Mode != 0){
				Drone.target.pitch_target = Esp.target_pitch*1.5f;
				Drone.target.roll_target = Esp.target_roll*1.5f;
				Drone.target.yaw_target = Esp.target_yaw;
				
				Drone.target.speed_y_target = -Esp.target_pitch/4.0f;
				Drone.target.speed_x_target = Esp.target_roll/4.0f;
				
//				Drone.Init_PWM = Esp.target_height*2000;
//				if(Drone.Init_PWM > 9000){Drone.Init_PWM = 9000;}
			}
			else{
				Drone.target.pitch_target = 0;
				Drone.target.roll_target = 0;
			}
		}
	}
}


/********************************************************************************
 * @brief  	无人机通讯函数
 
 * @param  	无人机通讯函数
 
 * @retval 	
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t uwb_send_buffer[84] = {0};
extern float X_HM_temp,Y_HM_temp,yaw_temp;//test
uint8_t GearDrone::Communication(uwb_struct_data *Uwb_temp){
	
	uint8_t state;
	
	/* UWB通讯部分，向其他无人机发送自身的公共数据 */
	public_data.id = UWB_ID;
//	public_data.network_time = current_data.network_time;
	public_data.pos_x = current_data.pos_x;
	public_data.pos_y = current_data.pos_y;
	public_data.pos_z = current_data.pos_z;
	public_data.x_speed = current_data.x_speed;
	public_data.y_speed = current_data.y_speed;
	public_data.z_speed = current_data.z_speed;
//	public_data.x_acc = current_data.x_acc;
//	public_data.y_acc = current_data.y_acc;
//	public_data.z_acc = current_data.z_acc;
	
	for(int i=0;i<5;i++){
		public_data.distance[i] = current_data.stoo_distance[i];
		public_data.velocity[i] = current_data.stoo_velocity[i];
	}
	
	memcpy(uwb_send_buffer,&public_data,68);
	state = HAL_UART_Transmit_DMA(&huart2,uwb_send_buffer,68);
	if(Esp.Init_State == 2 && Drone.Drone_Mode != 5){
			Esp.ESP12F_Send(Drone.current_data.x_speed,current_data.roll,Flow.sum_flow_x,Drone.Flash_ID,Drone.System_time_ms,1);
	}
	return state;
	
}

