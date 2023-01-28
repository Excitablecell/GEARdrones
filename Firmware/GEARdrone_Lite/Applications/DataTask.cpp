/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @author  EXcai
  * @brief   无人机数据传输和处理
  *
  ==============================================================================
													 该文件包含以下内容
  ==============================================================================
    @note
			-UWB数据处理
			-陀螺仪数据处理
			-光流传感器数据处理
			-遥控器、matlab上位机数据处理

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

extern GearDrone Drone;
extern struct Mpu6050_Data Mpu6050;

extern uint8_t optic_flow[9];
extern int16_t flow_x,flow_y;
extern float pos_x_flow, pos_y_flow;
extern uint16_t flow_timespan[2];
extern uint8_t data[100];

/* 巴特沃斯低通滤波器系数 */ 
static float IIRCoeffs32LP[5] = {1.0f, 2.0f, 1.0f, 1.561f,-0.641f};//2HZ低通
static float IIRCoeffs32LP2[5] = {1.0f, 2.0f, 1.0f, 1.307285f,-0.4918f};//20HZ低通
/*滤波*/
MeanFilter<2> MDF_gyro[3];
LowPassFilter LF_x(0.85),LF_y(0.85);
MeanFilter<3> X_MDF,Y_MDF;
ComplementaryFilter CF_speed_flow[2] = {ComplementaryFilter(0.8),ComplementaryFilter(0.8)};
ComplementaryFilter CF_speed_uwb[2] = {ComplementaryFilter(0.8),ComplementaryFilter(0.8)};

ButterworthFilter BWF[3]={	ButterworthFilter(IIRCoeffs32LP,0.0201f,1),
														ButterworthFilter(IIRCoeffs32LP,0.0201f,1),
														ButterworthFilter(IIRCoeffs32LP,0.0201f,1)	};
ButterworthFilter BWF_ACC[3]={	ButterworthFilter(IIRCoeffs32LP2,0.0461318,1),
																ButterworthFilter(IIRCoeffs32LP2,0.0461318,1),
																ButterworthFilter(IIRCoeffs32LP2,0.0461318,1)	};

#ifdef __cplusplus    
extern "C" {         
#endif

extern uint32_t Get_sys_time_ms(void);
extern void FlashWriteBuff( const uint32_t destination_address, uint8_t *const buffer,uint32_t length );
extern void FlashReadBuff(const uint32_t source_address,uint8_t *const buffer,uint16_t length);

uint8_t remote_command;
static char *msg_matlab,*token;

#ifdef __cplusplus
}
#endif

UWB Uwb;
extern void angular_velocity_controller(uint8_t controller_state);

/************************************************************
 * @brief  	UWB工作函数
 
 * @param  	通过二值信号量传递uwb解包
 * @retval 	
 * @author EXcai(亢体造梦)
 ***********************************************************/
void Uwbunpack(void const * argument){
	
	/* Sensors data structure ----------------------------------------------------*/
	int remote_z_pos_target;
	const char s[2] = ";";
	for(;;){
		if(xSemaphoreTake(uwb_Semaphore, portMAX_DELAY) == pdTRUE){
		
			taskENTER_CRITICAL();//临界区
			Uwb.User_Unpack();
			msg_matlab = strchr((char*)Uwb.computer_command.data,(int)'>');//"vel>2.0;2.0;3.0;"
			if(msg_matlab != NULL){
				#if MATLAB_SECOND_OREDER_CONTROL
				msg_matlab = &msg_matlab[1];
				token = strtok(msg_matlab, s);
				Drone.acc_target[0] = atof(token);
				token = strtok(NULL, s);
				Drone.acc_target[1] = -atof(token);
				token = strtok(NULL, s);
				Drone.acc_target[2] = atof(token);
				#else
				msg_matlab = &msg_matlab[1];
				token = strtok(msg_matlab, s);
				Drone.target_x_vel = atof(token);
				token = strtok(NULL, s);
				Drone.target_y_vel = atof(token);
				token = strtok(NULL, s);
				Drone.target_z_vel = atof(token);
				#endif
			}
			taskEXIT_CRITICAL();//退出临界区
			
			#if USE_UWB
			if(Uwb.anchor_count > 2){
				Uwb.system_time = Drone.systemtime_10ms;
				Drone.x_pos = -1*(Uwb.pos_3d[0]);
				Drone.y_pos = Uwb.pos_3d[1];
				Drone.x_vel = Uwb.vel_3d_estimate[0]*-10.0f;
				Drone.y_vel = Uwb.vel_3d_estimate[1]*10.0f;
			}
			if(Drone.systemtime_10ms < 800){
				Drone.init_x_pos = -1*Uwb.pos_3d[0];
				Drone.init_y_pos = Uwb.pos_3d[1];
			}
			#endif

			if(Uwb.remote_data.data_length != 0 && Uwb.remote_status != 0){
				if(1/*msg_matlab == NULL*/){
					if(abs(Uwb.remote_data.data[0] - 5) < 6){Drone.target_x_vel = float(Uwb.remote_data.data[0] - 5)*-4;}else{Drone.target_x_vel = 0;}
					if(abs(Uwb.remote_data.data[1] - 5) < 6){Drone.target_y_vel = float(Uwb.remote_data.data[1] - 5)*4;}else{Drone.target_y_vel = 0;}
				}
				if(Drone.yaw_target_angle > -50 && Drone.z_pos > 0.1f){Drone.yaw_target_angle = Drone.yaw_target_angle - 0.0015f;}
				if(Uwb.remote_data.data[2] > 4){
					remote_z_pos_target += (Uwb.remote_data.data[2] - 1);
					remote_command = 1;
				}
//			if(abs(Uwb.remote_data.data[3] - 5) < 6){yaw_target_angle = float(Uwb.remote_data.data[3] - 5)*2;}
					
				if(remote_z_pos_target > 800){remote_z_pos_target = 800;}
				if(remote_z_pos_target < 0){remote_z_pos_target = 0;}
				if(Uwb.remote_data.data[2] < 4 && remote_command == 1){
					remote_command = 2;
					Drone.target_z_vel = 0;
				}
//				if(Uwb.remote_data.data[2] == 0 && (Drone.Drone_Mode == 4 || Drone.Drone_Mode == 3 || Drone.Drone_Mode == 2)){
//					remote_command = 3;
//					Drone.target_z_vel = 0;
//				}
			}
		}
		vTaskSuspend(NULL);//挂起任务，直到下一次中断
	}
}
/**********************************************************
 * @brief  	mpu6050工作函数
 
 * @param  	mpu6050解包
 * @retval 	
 * @author EXcai(亢体造梦)
 **********************************************************/
uint8_t acc_write[2] = {5,6},acc_read[2];
void Mpuunpack(void const * argument){
	
  /* USER CODE BEGIN StartGoTask */
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 4;
  xLastWakeTime = xTaskGetTickCount();
	float x_acc_temp[2],y_acc_temp[2],z_acc_temp[2];
//	float flash_params[9] = {0.9988,0.9813,1.0016,-0.0071,-0.0072,-0.0768,0,0,0};
//	FlashWriteBuff(0x8014000,(uint8_t *)flash_params,36);  // 写入数据到Flash
  FlashReadBuff(0x8014000,(uint8_t *)Mpu6050.flash_params,36);  // 从Flash中读取数
	
	if(fabsf(Mpu6050.flash_params[3]) > 0.5f){
		memset(Mpu6050.flash_params,0,36);
		float ones[3] = {1.0f,1.0f,1.0f};
		memcpy(Mpu6050.flash_params,ones,12);
	}
  /* Infinite loop */
	for(;;){
		taskENTER_CRITICAL();//临界区
		/*读取陀螺仪角速度*/
		MPU_Get_Gyroscope(Mpu6050.gyro);
		/*读取加速度*/
		MPU_Get_Accelerometer(Mpu6050.acc);
		taskEXIT_CRITICAL();//退出临界区
		
		if(Drone.systemtime_10ms < 600){
			Mpu6050.gyro_init[0] = MDF_gyro[2].f(Mpu6050.gyro[0]/16.4);//Drone.roll
			Mpu6050.gyro_init[1] = MDF_gyro[0].f(Mpu6050.gyro[1]/16.4);//Drone.pitch
			Mpu6050.gyro_init[2] = MDF_gyro[1].f(Mpu6050.gyro[2]/16.4);//Drone.yaw
		}
		
		Drone.pitch_speed = MDF_gyro[0].f(Mpu6050.gyro[1]/16.4) - Mpu6050.gyro_init[1];
		Drone.roll_speed = MDF_gyro[2].f(Mpu6050.gyro[0]/16.4) - Mpu6050.gyro_init[0];
		Drone.yaw_speed = MDF_gyro[1].f(Mpu6050.gyro[2]/16.4) - Mpu6050.gyro_init[2];
		
		/*转换为大地坐标*/		
		x_acc_temp[0] = Mpu6050.flash_params[0] * Mpu6050.acc[0]/16384.0f + Mpu6050.flash_params[3];
		y_acc_temp[0] = Mpu6050.flash_params[1] * Mpu6050.acc[1]/16384.0f + Mpu6050.flash_params[4];
		z_acc_temp[0] = Mpu6050.flash_params[2] * Mpu6050.acc[2]/16384.0f + Mpu6050.flash_params[5];
		
		/*Drone.pitch*/y_acc_temp[1] = BWF_ACC[0].f(-1.0f*(cosf((Drone.yaw*PI)/180.0f)*cosf((Drone.pitch*PI)/180.0f)*(x_acc_temp[0])
										+ (sinf((Drone.roll*PI)/180.0f)*sinf((Drone.pitch*PI)/180.0f)*cosf((Drone.yaw*PI)/180.0f) - cosf((Drone.roll*PI)/180.0f)*sinf((Drone.yaw*PI)/180.0f))*(y_acc_temp[0])
										+ (sinf((Drone.roll*PI)/180.0f) * sinf((Drone.yaw*PI)/180.0f) + cosf((Drone.roll*PI)/180.0f) * sinf((Drone.pitch*PI)/180.0f) * cosf((Drone.yaw*PI)/180.0f))*(z_acc_temp[0])));
		
		/*Drone.roll*/x_acc_temp[1] = BWF_ACC[1].f(-1.0f*(sinf((Drone.yaw*PI)/180.0f)*cosf((Drone.pitch*PI)/180.0f)*(x_acc_temp[0])
		                 + (sinf((Drone.roll*PI)/180.0f)*sinf((Drone.pitch*PI)/180.0f)*sinf((Drone.yaw*PI)/180.0f) + cosf((Drone.roll*PI)/180.0f)*cosf((Drone.yaw*PI)/180.0f))*(y_acc_temp[0])
										 + (cosf((Drone.roll*PI)/180.0f) * sinf((Drone.yaw*PI)/180.0f) * sinf((Drone.pitch*PI)/180.0f) - sinf((Drone.roll*PI)/180.0f) * cosf((Drone.yaw*PI)/180.0f))*(z_acc_temp[0])));
		
		/*Z*/z_acc_temp[1] = BWF_ACC[2].f(- 1*sinf((Drone.pitch*PI)/180.0f)*(x_acc_temp[0])
								 + sinf((Drone.roll*PI)/180.0f)*cosf((Drone.pitch*PI)/180.0f)*(y_acc_temp[0])
								 + cosf((Drone.roll*PI)/180.0f)*cosf((Drone.pitch*PI)/180.0f)*(z_acc_temp[0])
								 - 1.0f);
		
		if(Drone.systemtime_10ms < 600){
			Mpu6050.acc_init[0] = x_acc_temp[1];
			Mpu6050.acc_init[1] = y_acc_temp[1];
			Mpu6050.acc_init[2] = z_acc_temp[1];
		}
		
		/*转换为m/ss*/	
		Drone.x_acc = (x_acc_temp[1] - Mpu6050.acc_init[0])*9.8;
		Drone.y_acc = (y_acc_temp[1] - Mpu6050.acc_init[1])*9.8;
		Drone.z_acc = (z_acc_temp[1] - Mpu6050.acc_init[2])*9.8;
		
		Mpu6050.x_speed_mpu += Drone.x_acc*(xFrequency*0.001);
		Mpu6050.y_speed_mpu += Drone.y_acc*(xFrequency*0.001);
		Mpu6050.z_speed_mpu += Drone.z_acc*(xFrequency*0.001);
		
		if(Drone.Drone_Mode != 0){
			Drone.angular_velocity_controller(Drone.Controller_State);
		}
		
		
		osDelayUntil(&xLastWakeTime,xFrequency);
	}
}

/************************************************************
 * @brief  	光流传感器工作函数
 
 * @param  	通过二值信号量传递光流传感器解包
 * @retval 	
 * @author EXcai(亢体造梦)
 ***********************************************************/
void Flowunpack(void const * argument){
	float vx_temp,vy_temp;
	float roll_speed_wait,pitch_speed_wait;//同步光流和陀螺仪的相位
	/* Sensors data structure ----------------------------------------------------*/
	for(;;){
		if(xSemaphoreTake(flow_Semaphore, portMAX_DELAY) == pdTRUE){
			
			/*读取光流数据并转换*/
			if(optic_flow[0] == 0xFE){
				uint8_t Check_sum=(uint8_t)(optic_flow[2]+optic_flow[3]+optic_flow[4]+optic_flow[5]);
				if(Check_sum == optic_flow[6]){
					flow_x = (int16_t)(optic_flow[3]<<8)|optic_flow[2];//Drone.roll 右移+
					flow_y = (int16_t)(optic_flow[5]<<8)|optic_flow[4];
					
					//同步相位
					X_MDF << Drone.roll_speed;
					X_MDF >> roll_speed_wait;
					Y_MDF << Drone.pitch_speed;
					Y_MDF >> pitch_speed_wait;
					
					//经过低通滤波
					LF_x << flow_x + 0.26*roll_speed_wait;
					LF_y << flow_y - 0.26*pitch_speed_wait;
					LF_x >> vx_temp;//Drone.roll 右移+
					LF_y >> vy_temp;
					
					if(fabs(vx_temp)< 500){
						if(Drone.z_pos < 0.06f){Drone.x_vel = 0;}
						#if (USE_UWB == 0)
						else{Drone.x_vel = CF_speed_flow[0].f(Drone.z_pos*(vx_temp/flow_timespan[1]),Drone.x_acc,flow_timespan[1]*0.001);}
						#endif
						
					}
					if(fabs(vy_temp)< 500){
						if(Drone.z_pos < 0.06f){Drone.y_vel = 0;}
						#if (USE_UWB == 0)
						else{Drone.y_vel = CF_speed_flow[1].f(Drone.z_pos*(vy_temp/flow_timespan[1]),Drone.y_acc,flow_timespan[1]*0.001);}
						#endif
					}
					
					//积分速度得到光流估计位置
					if(Drone.z_pos > 0.07f){
						pos_x_flow += Drone.x_vel*(flow_timespan[1]*0.001);
						pos_y_flow += Drone.y_vel*(flow_timespan[1]*0.001);
						
						#if (USE_UWB == 0)
						Drone.x_pos = pos_x_flow*11;
						Drone.y_pos = pos_y_flow*11;
						#endif
					}
				}
			}
		}
	vTaskSuspend(NULL);//挂起任务，直到下一次中断
	}
}

/************************************************************
 * @brief  	matlab遥控数据处理
 
 * @param  	matlab遥控数据赋值
 * @retval 	
 * @author EXcai(亢体造梦)
 ***********************************************************/
void Data_Task(void const * argument)
{
  /* USER CODE BEGIN StartGoTask */
	const char s[2] = ";";
  /* Infinite loop */
  for(;;)
  {
		
		/* matlab遥控数据接收 */
		if(xSemaphoreTake(esp_Semaphore, portMAX_DELAY) == pdTRUE){
			taskENTER_CRITICAL();//临界区
			msg_matlab = strchr((char*)data,(int)'>');//"vel>2.0;2.0;3.0;"
			if(msg_matlab != NULL){
				#if MATLAB_SECOND_OREDER_CONTROL
				msg_matlab = &msg_matlab[1];
				token = strtok(msg_matlab, s);
				Drone.acc_target[0] = atof(token);
				token = strtok(NULL, s);
				Drone.acc_target[1] = -atof(token);
				token = strtok(NULL, s);
				Drone.acc_target[2] = atof(token);
				#else
				msg_matlab = &msg_matlab[1];
				token = strtok(msg_matlab, s);
				//Drone.target_x_vel = atof(token);
				token = strtok(NULL, s);
				//Drone.target_y_vel = atof(token);
				token = strtok(NULL, s);
				//Drone.target_z_vel = atof(token);
				#endif
			}
//		memset(data,0,100);
		taskEXIT_CRITICAL();//退出临界区
		}
		vTaskSuspend(NULL);//挂起任务，直到下一次中断
	}
  /* USER CODE END StartGoTask */
}
