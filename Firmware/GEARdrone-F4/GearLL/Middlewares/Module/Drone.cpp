/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Drone.cpp
  * @author  EXcai
  * @brief   无人机类，包含各种抽象操作和状态
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
		- 参考
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
#include "Drone.h"

/* Private variables --------------------------------------------------------*/
static float loss_tof_height = 0;
static MeanFilter<3> MDF_yaw;
static MeanFilter<6> MF_pitch,MF_roll;
static MeanFilter<100> MF_acc_init[3];
static MedianFilter<3> MDF_x_speed,MDF_y_speed;
KalmanFilter KF_z_speed(0.005,0.1,0.6),KF_x_speed(0.005,0.1,0.6),KF_y_speed(0.005,0.1,0.6);
ComplementaryFilter2 z_speed_fusion(0.08,0.02),x_speed_fusion(0.15,0.02),y_speed_fusion(0.15,0.02);
/* function prototypes -------------------------------------------------------*/

/********************************************************************************
 * @brief  	无人机数据采集函数
 
 * @param  	以5ms周期从各个传感器汇总数据
 
 * @retval 	0: 成功
						1: 启动失败
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
float X_HM_temp,Y_HM_temp,yaw_temp;
uint8_t GearDrone::Get_Sensors_Data(pressure_data *Spl_temp,mag_data *Mag_temp,tof_data *Tof_temp,flow_data *Flow_temp,imu_data *Imu_temp,adc_data *Adc_temp,uwb_struct_data *Uwb_temp){
	
	/* Sensors data structure ----------------------------------------------------*/
	
	/* Private variables --------------------------------------------------------*/
	uint8_t Sta = 0;
	
	/* Filters ------------------------------------------------------------------*/
	
	/* RTOS Resources -----------------------------------------------------------*/
//	xQueueReceive(Spl_Port,&Spl_temp,0);
//	xQueueReceive(Mag_Port,&Mag_temp,0);
//	xQueueReceive(Tof_Port,&Tof_temp,0);
//	xQueueReceive(Flow_Port,&Flow_temp,0);
//	xQueueReceive(Imu_Port,&Imu_temp,0);
//	xQueueReceive(Adc_Port,&Adc_temp,0);
	System_time_ms = Get_SystemTimer()/1000;
	real_time = System_time_ms/1000;
	period_beat++;
	
	/* 任务逻辑 ------------------------------------------------------------------*/
	if(System_time_ms < 3000){
		init_data.network_time = Uwb_temp->system_time;
		UWB_ID = Uwb_temp->id;
		init_data.pitch = Imu_temp->pitch;
		init_data.roll = Imu_temp->roll;
		init_data.imu_yaw = Imu_temp->yaw;
		MF_acc_init[0] << Imu_temp->x_acc;
		MF_acc_init[0] >> init_data.x_acc;
		MF_acc_init[1] << Imu_temp->y_acc;
		MF_acc_init[1] >> init_data.y_acc;
		MF_acc_init[2] << Imu_temp->z_acc;
		MF_acc_init[2] >> init_data.z_acc;
		init_data.pitch_speed = Imu_temp->pitch_speed;
		init_data.roll_speed = Imu_temp->roll_speed;
		init_data.yaw_speed = Imu_temp->yaw_speed;
		
		init_data.altitude = Spl_temp->Altitude;
		init_data.pos_z = 0;
		
		init_data.VBAT = Adc_temp->BAT_Voltage;
		
		if(Uwb_temp->Mode == LP_Mode){
			init_data.pos_x = -Uwb_temp->pos_3d[1];
			init_data.pos_y = Uwb_temp->pos_3d[0];
		}
		else if(Uwb_temp->Mode == DR_Mode){
			for(int i=0;i<9;i++){init_data.stoo_distance[i] = Uwb_temp->distanse[i];}
		}
		
	}

	memcpy(&last_data,&current_data,sizeof(current_data));
	
	UWB_ID = Uwb_temp->id;
	current_data.timestamp_ms = Get_SystemTimer()/1000;
	current_data.VBAT = Adc_temp->BAT_Voltage;
	current_data.pitch = Imu_temp->pitch;
	current_data.roll = Imu_temp->roll;
	current_data.imu_yaw = Imu_temp->yaw;
	current_data.pitch_speed = Imu_temp->pitch_speed - init_data.pitch_speed;
	current_data.roll_speed = Imu_temp->roll_speed - init_data.roll_speed;
	current_data.yaw_speed = Imu_temp->yaw_speed - init_data.yaw_speed;
	current_data.x_acc = Imu_temp->x_acc - init_data.x_acc;
	current_data.y_acc = Imu_temp->y_acc - init_data.y_acc;
	current_data.z_acc = Imu_temp->z_acc - init_data.z_acc;
	current_data.altitude = Spl_temp->Altitude;
	current_data.altitude_speed = Spl_temp->Altitude_speed;

	X_HM_temp = Mag_temp->Y_HM*cosf(current_data.pitch*PI/180.0f) + Mag_temp->X_HM*arm_sin_f32(current_data.pitch*PI/180.0f)*arm_sin_f32(current_data.roll*PI/180.0f) - Mag_temp->Z_HM*cosf(current_data.roll*PI/180.0f)*arm_sin_f32(current_data.pitch*PI/180.0f);
	Y_HM_temp = Mag_temp->X_HM*cosf(current_data.roll*PI/180.0f) + Mag_temp->Z_HM*arm_sin_f32(current_data.roll*PI/180.0f);
	MDF_yaw << (atan2(Y_HM_temp,X_HM_temp) * (180.0f / PI) - 36)*2.0f;
	MDF_yaw >> yaw_temp;
	
	if(Y_HM_temp < 300){
		yaw_temp = (atan2(Y_HM_temp,X_HM_temp) * (180.0f / PI) - 36)*2.0f;
	}
	else{
		yaw_temp = -(atan2(Y_HM_temp,X_HM_temp) * (180.0f / PI) - 36)*2.0f;
	}

	//QMC5883没做完磁力计融合
//	/* 磁力计非线性补偿*/
//	if(yaw_temp < 19.0f){
//		yaw_temp *= 6.2f;
//	}
//	else if(yaw_temp < 260.0f && yaw_temp > 100.0f){
//		yaw_temp = yaw_temp*1.2f - 20.0f;
//	}
//	else{
//		yaw_temp = last_data.yaw + Imu_temp->yaw_speed/(current_data.timestamp_ms - last_data.timestamp_ms);
//	}	 
//	MDF_yaw << 0.4f*yaw_temp + 0.6f*(current_data.imu_yaw - last_data.imu_yaw + last_data.yaw);
//	MDF_yaw >> current_data.yaw;
	current_data.yaw = Imu_temp->yaw;

	if(period_beat%4 == 0){//每20ms融合一次
		Sta = Fusion_Height(0,Spl_temp,Tof_temp,Imu_temp);
		Sta = Fusion_Mov(0,Imu_temp,Flow_temp,Uwb_temp);
	}
	
	return Sta;
}
/********************************************************************************
 * @brief  	无人机高度融合函数
 
 * @param  	融合多个数据源以得到准确宽范围高度
 
 * @retval 	0: 成功
						1: 失败
 * @author	EXcai(亢体造梦)
 *******************************************************************************/

uint8_t GearDrone::Fusion_Height(uint8_t mode,pressure_data *altitude,tof_data *height,imu_data *mpu){
	/* Private variables --------------------------------------------------------*/
	uint8_t Sta = 0;
	float height_temp,acc_kal_temp,height_kal_temp,acc_speed_temp,altitude_height;
	
	/*确定高度*/
	height_temp = height->TOF_Height*arm_cos_f32(fabs((current_data.roll/180.0f)*PI))*arm_cos_f32(fabs((current_data.pitch/180.0f)*PI));
	if(height_temp < 1.5f){
		current_data.pos_z = height_temp;
		loss_tof_height = current_data.altitude - init_data.altitude;
	}
	else{
		current_data.pos_z = current_data.altitude - init_data.altitude;
	}
	
	/*确定z方向速度,m/s*/
	acc_kal_temp = mpu->z_acc*9.8f;
	height_kal_temp = (height->TOF_Speed)*0.001f;
	
//	current_data.z_speed = KF_z_speed.f(height_kal_temp,acc_kal_temp);//感觉误差不是高斯分布的
	current_data.z_speed = z_speed_fusion.f(height_kal_temp,acc_kal_temp);
	if(fabsf(current_data.z_speed)>4){//速度测量上限
		current_data.z_speed = last_data.z_speed;
	}
	return Sta;
}

/********************************************************************************
 * @brief  	无人机平移融合函数
 
 * @param  	融合多个数据源以得到准确平移速度
 
 * @retval 	0: 成功
						1: 失败
 * @author	EXcai(亢体造梦)
 *******************************************************************************/

uint8_t GearDrone::Fusion_Mov(uint8_t mode,imu_data *mpu,flow_data *flow,uwb_struct_data *Uwb_temp){
	uint8_t Sta = 0;
	float x_speed_temp,y_speed_temp;
	
	/*同步相位，抵消摇动*/
	MF_pitch << current_data.pitch_speed;
	MF_pitch >> pitch_speed_wait;
	MF_roll << -current_data.roll_speed;
	MF_roll >> roll_speed_wait;
	
	MDF_x_speed << flow->flow_x;
	MDF_y_speed << flow->flow_y;
	MDF_x_speed >> x_speed_temp;
	MDF_y_speed >> y_speed_temp;
	
	/*光流部分,机体坐标系下的速度*/
	x_speed_temp = (flow->flow_x*current_data.pos_z*0.01)/(flow->time_span_ms/1000.0) - 0.02*roll_speed_wait;//
	y_speed_temp = (flow->flow_y*current_data.pos_z*0.01)/(flow->time_span_ms/1000.0) - 0.02*pitch_speed_wait;//
	
	Flow.sum_flow_x += x_speed_temp*0.02f;
	Flow.sum_flow_y += y_speed_temp*0.02f;
	
	if(current_data.pos_z < 0.3f){
		Flow.sum_flow_x = 0;
		Flow.sum_flow_y = 0;
	}
	
	/*已注销内容：平移速度转换至导航坐标系下*/
//	MF_x_speed << x_speed_temp*cosf(-(current_data.yaw*PI)/180.0f) - y_speed_temp*arm_sin_f32(-(current_data.yaw*PI)/180.0f);
//	MF_y_speed << x_speed_temp*arm_sin_f32(-(current_data.yaw*PI)/180.0f) + y_speed_temp*cosf(-(current_data.yaw*PI)/180.0f);
	
	/*平移速度保持在机体坐标系下*/
	
	/*限幅，去除毛刺*/
	if(fabsf(x_speed_temp) < 3){
		current_data.x_speed = x_speed_fusion.f(x_speed_temp,mpu->x_acc*9.8f);
//		current_data.x_speed = x_speed_temp;
	}
	if(fabsf(y_speed_temp) < 3){
		current_data.y_speed = y_speed_fusion.f(y_speed_temp,mpu->y_acc*9.8f);
//		current_data.y_speed = y_speed_temp;
	}
	
	/*已注销内容：和加速度计进行卡尔曼融合*/
//	current_data.x_speed = KF_x_speed.f(MF_x_speed.f(x_speed_local*cosf(-(current_data.yaw*PI)/180.0f) - y_speed_local*arm_sin_f32(-(current_data.yaw*PI)/180.0f)),mpu->x_acc*9.8f);
//	current_data.y_speed = KF_y_speed.f(MF_y_speed.f(x_speed_local*arm_sin_f32(-(current_data.yaw*PI)/180.0f) + y_speed_local*cosf(-(current_data.yaw*PI)/180.0f)),mpu->y_acc*9.8f);
	
	/*UWB部分*/
	if(Uwb_temp->Mode == LP_Mode){
		current_data.pos_x = -Uwb_temp->pos_3d[1];
		current_data.pos_y = Uwb_temp->pos_3d[0];
		
//		current_data.x_speed = Uwb_temp->vel_3d[0];
//		current_data.y_speed = Uwb_temp->vel_3d[1];
	}
	else if(Uwb_temp->Mode == DR_Mode){
		for(int i=0;i<5;i++){
			current_data.stoo_distance[i] = Uwb_temp->distanse[i];
			if(current_data.network_time != Uwb_temp->system_time){//避免DR模式下函数执行周期不同出现错误的相对速度
				current_data.stoo_velocity[i] = (Uwb_temp->distanse[i] - last_data.stoo_distance[i])*1000.0f/(Uwb_temp->system_time - current_data.network_time);
			}
			else{
				current_data.stoo_velocity[i] = 0;
			}
		}
	}	
	for(int i=0;i<5;i++){//获取其他飞机的公共数据(!!!此处memcpy可能出现长度错误)
		memcpy(&others_public_data[i],Uwb_temp->others_data[i].data,68);
	}
	current_data.network_time = Uwb_temp->system_time;
	
	return Sta;
}

/********************************************************************************
 * @brief  	无人机简易动力学模型
 
 * @param  	无人机简易动力学模型
 
 * @retval 	0: 成功
						1: 失败
 * @author	EXcai(亢体造梦)
 *******************************************************************************/

uint8_t GearDrone::Basic_Dynamic_Model(){
	
	/*简易动力学模型*/
	LF_PWM = 	Pitch_Speed_PID.Out - Yaw_Speed_PID.Out + Roll_Speed_PID.Out + Height_Speed_PID.Out + Init_PWM;
	RF_PWM = 	Pitch_Speed_PID.Out + Yaw_Speed_PID.Out - Roll_Speed_PID.Out + Height_Speed_PID.Out + Init_PWM;
	LB_PWM = -Pitch_Speed_PID.Out + Yaw_Speed_PID.Out + Roll_Speed_PID.Out + Height_Speed_PID.Out + Init_PWM;
	RB_PWM = -Pitch_Speed_PID.Out - Yaw_Speed_PID.Out - Roll_Speed_PID.Out + Height_Speed_PID.Out + Init_PWM;
	
	if(Height_Speed_PID.Out + Init_PWM > 11000){
		LF_PWM = 	Pitch_Speed_PID.Out - Yaw_Speed_PID.Out + Roll_Speed_PID.Out + 11000;
		RF_PWM = 	Pitch_Speed_PID.Out + Yaw_Speed_PID.Out - Roll_Speed_PID.Out + 11000;
		LB_PWM = -Pitch_Speed_PID.Out + Yaw_Speed_PID.Out + Roll_Speed_PID.Out + 11000;
		RB_PWM = -Pitch_Speed_PID.Out - Yaw_Speed_PID.Out - Roll_Speed_PID.Out + 11000;
	}
	
	/*PWM占空比上下限幅*/
	if(LF_PWM > 20000){LF_PWM = 20000;}
	if(RF_PWM > 20000){RF_PWM = 20000;}
	if(LB_PWM > 20000){LB_PWM = 20000;}
	if(RB_PWM > 20000){RB_PWM = 20000;}
	if(LF_PWM <= 0){LF_PWM = 0;}
	if(RF_PWM <= 0){RF_PWM = 0;}
	if(LB_PWM <= 0){LB_PWM = 0;}
	if(RB_PWM <= 0){RB_PWM = 0;}
	
	
	return 0;
}

/********************************************************************************
 * @brief  	无人机控制器
 
 * @param  	无人机控制器
 
 * @retval 	0: 成功
						1: 失败
 * @author	EXcai(亢体造梦)
 *******************************************************************************/
uint8_t GearDrone::Controller(uint8_t status){
	
	Pitch_PID.SetPIDParam(6.5,0.02,0,100,2000);//
	Pitch_Speed_PID.SetPIDParam(55,0,6,800,20000);//
	Pitch_PID.I_SeparThresh = 10; 

	Roll_PID.SetPIDParam(7,0.02,0,100,2000);//
	Roll_Speed_PID.SetPIDParam(60,0,6,800,20000);//
	Roll_PID.I_SeparThresh = 10; 

	Yaw_PID.SetPIDParam(8.0f,0,0,2,120);//
	Yaw_Speed_PID.SetPIDParam(45,0,5,500,8000); //

	Height_PID.SetPIDParam(0.75f,0.15f,0.45f,5,3.5);//(0.65f ,0.3f,0.55 ,5 ,3.5);
	Height_PID.I_SeparThresh = 0.15;
	Height_Speed_PID.SetPIDParam(12000, 2000, 0, 3000, 4500);//(10000, 0, 0, 400, 7000);
	
	Speed_X_PID.SetPIDParam(4.5,0.05,0,9,10);//20 0.01 0.05
	Speed_Y_PID.SetPIDParam(5,0.07,0,9,10);//20 0.01 0.05
	
	Pos_X_PID.SetPIDParam(0.5,0.1,0.01,2,1);
	Pos_Y_PID.SetPIDParam(0.5,0.1,0.01,2,1);
	
	/*简易串级控制器模型*/
	if(period_beat%2 == 0){//以10ms为周期进行速度和位置控制
		if(status == 1){//1:所有控制启动
			if(target.speed_y_target == 0 && target.speed_x_target == 0){
		//		if(Uwb.system_time != 0 && target.speed_y_target == 0 && target.speed_x_target == 0){
				Pos_X_PID.Current = Flow.sum_flow_x;
				Pos_X_PID.Adjust();
				Speed_X_PID.Target = Pos_X_PID.Out;
				
				Pos_Y_PID.Current = Flow.sum_flow_y;
				Pos_Y_PID.Adjust();
				Speed_Y_PID.Target = Pos_Y_PID.Out;
			}
			else{
				Pos_X_PID.Target = Flow.sum_flow_x;
				Pos_Y_PID.Target = Flow.sum_flow_y;
				Speed_X_PID.Target = target.speed_x_target;
				Speed_Y_PID.Target = target.speed_y_target;
			}
			
			Speed_X_PID.Current = current_data.x_speed;
			Speed_X_PID.Adjust();
			Roll_PID.Target = Speed_X_PID.Out;
		
			Speed_Y_PID.Current = current_data.y_speed;
			Speed_Y_PID.Adjust();
			Pitch_PID.Target = -Speed_Y_PID.Out;
		}
		else{
			Pitch_PID.Target = target.pitch_target;
			Roll_PID.Target = target.roll_target;
		}
	}

	Pitch_PID.Current = current_data.pitch;
	Pitch_PID.Adjust();
	Pitch_Speed_PID.Target = Pitch_PID.Out;
	Pitch_Speed_PID.Current = current_data.pitch_speed;
	Pitch_Speed_PID.Adjust();
	
	Roll_PID.Current = current_data.roll;
	Roll_PID.Adjust();
	Roll_Speed_PID.Target = Roll_PID.Out;
	Roll_Speed_PID.Current = current_data.roll_speed;
	Roll_Speed_PID.Adjust();
	
	Yaw_PID.Target = target.yaw_target;
	Yaw_PID.Current = current_data.yaw;
	Yaw_PID.Adjust();
	Yaw_Speed_PID.Target = Yaw_PID.Out;
	Yaw_Speed_PID.Current = current_data.yaw_speed;
	Yaw_Speed_PID.Adjust();

	if((status == 1 || status == 3) && period_beat%8 == 0){//1:所有控制启动; 3:只进行高度+姿态控制; 以40ms为周期进行速度和位置控制
		Height_PID.Target = target.pos_z_target;
		Height_PID.Current = current_data.pos_z;
		Height_PID.Adjust();
	
		Height_Speed_PID.Target = Height_PID.Out;
		Height_Speed_PID.Current = current_data.z_speed;
		Height_Speed_PID.Adjust();
	}
	
	if(status != 0){
	
	/*简易动力学模型*/
	Basic_Dynamic_Model();
	
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,LB_PWM);//LB_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,LF_PWM);//LF_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,RB_PWM);//RB_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,RF_PWM);//RF_PWM
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,Init_PWM);//LB_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,Init_PWM);//LF_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,Init_PWM);//RB_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,Init_PWM);//RF_PWM
	}
	#ifdef NO_MOTOR//不要转动螺旋桨
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);//LB_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);//LF_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,0);//RB_PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,0);//RF_PWM
	#endif
//	if(System_time_ms < 24000 && System_time_ms > 12000){
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,System_time_ms - 12000);//LB_PWM
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,System_time_ms - 12000);//LF_PWM
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,System_time_ms - 12000);//RB_PWM
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,System_time_ms - 12000);//RF_PWM
//	}
	return status;
}

/********************************************************************************
 * @brief  传入uwb测距数据 
 * @param  	无
 * @retval 	结果
 * @attention 传入uwb测距数据 
									
 * @author EXcai(亢体造梦)
 *******************************************************************************/

uint8_t GearDrone::NLR_Get_Data(float uwb_distances[7]){
	
	if(UWB_ID == 0){
		uwb_distances[0] = current_data.stoo_distance[3];
		uwb_distances[1] = current_data.stoo_distance[4];
		uwb_distances[2] = others_public_data[1].distance[3];
		uwb_distances[3] = others_public_data[1].distance[4];
		uwb_distances[4] = others_public_data[2].distance[3];
		uwb_distances[5] = others_public_data[2].distance[4];
		uwb_distances[6] = others_public_data[3].distance[4];
	}
	
	return 0;
}
