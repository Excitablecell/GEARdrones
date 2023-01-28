/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @author  EXcai
  * @brief   无人机状态上报
  *
  ==============================================================================
													 该文件包含以下内容
  ==============================================================================
    @note
			-无人机状态上报任务

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

  ********************************************************************************/

#include "Drone.h"

#if USE_MAGSENSOR
#include "HMC5883L.h"
mag_data mag;
#endif

#if USE_BAROMETER
#include "SPL06_001.h"
SPL06 Pressure_sensor;
#endif

extern GearDrone Drone;
extern UWB Uwb;

void Upper_Monitor(void const * argument)
{
  /* USER CODE BEGIN Upper_Monitor */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
		
		if(Drone.systemtime_10ms > 800){
			taskENTER_CRITICAL();//临界区
			#if (SEND_TYPE == 0)	//发送格式 GEARmonitor_MQTT
				printf_dma(&huart3,MQTT_TEMPLATE,Uwb.id,Drone.systemtime_10ms,Drone.target_z_pos,0.0f,Drone.z_pos);
			#elif (SEND_TYPE == 1)	//发送格式 matlab 单机
					#if MATLAB_SECOND_OREDER_CONTROL
					printf_dma(&huart3,"AT+MQTTPUB=0,\"MAV%u_TX2\",\"%.2f %.2f %.2f %d %.2f %.2f %.2f\",1,0\r\n",Uwb.id,Drone.x_pos,Drone.y_pos,Drone.z_pos,Drone.systemtime_10ms,x_vel,y_vel,z_vel);
					#else
					printf_dma(&huart3,"AT+MQTTPUB=0,\"MAV%u_TX2\",\"%.2f %.2f %.2f %d %.2f\",1,0\r\n",Uwb.id,Drone.x_pos,Drone.y_pos,Drone.z_pos,Drone.systemtime_10ms,yaw);
					#endif
			#elif (SEND_TYPE == 2)	//发送格式 matlab 多机
			printf_dma(&huart3,"AT+MQTTPUB=0,\"MAV%u_TX3\",\"%.2f %.2f %.2f %.2f %.2f %.2f %d\",1,0\r\n",Uwb.id,Drone.x_pos,Drone.y_pos,Drone.z_pos,cosf(0.003*(Drone.systemtime_10ms - start_time_10ms)),sinf(0.003*(Drone.systemtime_10ms - start_time_10ms)),0.4f,Drone.systemtime_10ms);
			#elif (SEND_TYPE == 3)	//发送格式 uwb 多机
			static float uwb_send_buffer[3];
			uwb_send_buffer[0] = Drone.x_pos;
			uwb_send_buffer[1] = Drone.y_pos;
			uwb_send_buffer[2] = Drone.z_pos;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)uwb_send_buffer, 12);
			//osDelay(25);
			//printf_dma(&huart3,"%u %.2f %.2f %.2f %d\r\n",Uwb.id,(Pressure_sensor.altitude - Pressure_sensor.init_altitude),Drone.z_pos,Pressure_sensor.z_speed_barometer,Drone.systemtime_10ms);
			//printf_dma(&huart3,"%u %.2f %.2f %.2f %d",Uwb.id,Drone.x_pos,Drone.y_pos,Drone.z_pos,Drone.systemtime_10ms);
			#endif
			taskEXIT_CRITICAL();//退出临界区
		}
		osDelayUntil(&xLastWakeTime,xFrequency);
  }
  /* USER CODE END Upper_Monitor */
}
