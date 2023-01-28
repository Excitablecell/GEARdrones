/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @author  EXcai
  * @brief   无人机测量类传感器工作函数
  *
  ==============================================================================
													 该文件包含以下内容
  ==============================================================================
    @note
			-磁力计 tof 气压计等测量类传感器的工作和数据处理

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

#ifdef __cplusplus    
extern "C" {         
#endif

uint8_t tof_measure_status = 0,tof_init_status = 0,tof_devive_id = 0;
VL53L1_CalibrationData_t tof_calibration_data;
#ifdef __cplusplus
}
#endif

extern GearDrone Drone;
extern UWB Uwb;
ComplementaryFilter CF_tof_speed(0.9);

/* 二阶巴特沃斯低通滤波器系数*/ 
static float IIRCoeffs32LP[5] = {1.0f, 2.0f, 1.0f, 1.143f,-0.413f};
ButterworthFilter BWF_pos_z(IIRCoeffs32LP,0.0674f,1);

void TOF_Task(void const * argument)
{
  /* USER CODE BEGIN StartGoTask */
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();
	HAL_GPIO_WritePin(XSHUT_GPIO_Port,XSHUT_Pin,GPIO_PIN_SET);
	static float tof_height,last_height;
//	VL53Cali(&VL53,&tof_calibration_data);
	#if (TOF_SENSOR==0)
	tof_init_status = vl53l0x_init(&vl53l0x_dev);
	tof_devive_id = vl53l0x_set_mode(&vl53l0x_dev,3);
	if (tof_init_status == VL53L0X_ERROR_NONE){
		tof_init_status = VL53L0X_SetLimitCheckValue(&vl53l0x_dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.25*65536));
	}
	if (tof_init_status == VL53L0X_ERROR_NONE){
		tof_init_status = VL53L0X_SetLimitCheckValue(&vl53l0x_dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(32*65536));
	}
	if (tof_init_status == VL53L0X_ERROR_NONE){
		tof_init_status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53l0x_dev,20000);
	}
	#endif
	
	#if USE_MAGSENSOR
	float X_HM_temp,Y_HM_temp;
	/* 磁力计初始化 */
	Magsensor.Init((int16_t *)flash_params);
	#endif
	
	#if USE_BAROMETER
	Pressure_sensor.Sensor_state = Pressure_sensor.Init();
	#endif
  /* Infinite loop */
  for(;;)
  {
		last_height = Drone.z_pos;
		
		/* 磁力计magsensor */
		#if USE_MAGSENSOR
		//Magsensor.Calibration((int16_t *)flash_params,Drone.systemtime_10ms);
		mag = Magsensor.Get_Value();
		X_HM_temp = mag.Y_HM*cosf(pitch*PI/180.0f) + mag.X_HM*arm_sin_f32(pitch*PI/180.0f)*arm_sin_f32(roll*PI/180.0f) - mag.Z_HM*cosf(roll*PI/180.0f)*arm_sin_f32(pitch*PI/180.0f);
		Y_HM_temp = mag.X_HM*cosf(roll*PI/180.0f) + mag.Z_HM*arm_sin_f32(roll*PI/180.0f);
		
		if(Y_HM_temp < 300){mag.yaw = BWF_mag.f((-(atan2(Y_HM_temp,X_HM_temp) * (180.0f / PI)) - 118)*10.2f);}
		else{mag.yaw = BWF_mag.f(((atan2(Y_HM_temp,X_HM_temp) * (180.0f / PI)) - 118)*10.2f);}
		#endif
		
		/* 气压计barometer */
		#if USE_BAROMETER
		Pressure_sensor.Get_Value(xFrequency*0.001f);
		if(Drone.z_pos < 0.2f){Pressure_sensor.init_altitude = Pressure_sensor.altitude;}
		#endif
		
		/* 红外tof定高 */
		#if (TOF_SENSOR==0)
		tof_measure_status = vl53l0x_interrupt_start(&vl53l0x_dev,&vl53l0x_data,buf);
		tof_height = BWF_pos_z.f(0.001*Distance_data*cosf(fabs((Drone.roll/180.0f)*PI))*cosf(fabs((Drone.pitch/180.0f)*PI)));
		#else
		tof_measure_status = getDistance(&VL53);
		if(tof_measure_status == 0 && tof_distance_int32 != -1){tof_height = BWF_pos_z.f(0.001*(float)tof_distance_int32*cosf(fabs((Drone.roll/180.0f)*PI))*cosf(fabs((Drone.pitch/180.0f)*PI)));}//将测量距离换算成对地高度
		#endif
		
		/* 融合UWB高度 */
		#if USE_UWB	
		static float basic_diff;
		if(tof_height < 2.6f){
			Drone.z_pos = tof_height;
			basic_diff = fabsf(tof_height - (-Uwb.pos_3d[2]));
		}
		else{
			Drone.z_pos = -Uwb.pos_3d[2] + basic_diff;
		}
		Drone.z_vel = (Drone.z_pos - last_height)/(0.001*xFrequency);
		
		#else
		if(tof_height < 2.0f && tof_measure_status == 0 && tof_distance_int32 != -1){
			Drone.z_pos = tof_height;
			Drone.z_vel = (Drone.z_pos - last_height)/(0.001*xFrequency);
		}
		#endif
		
		osDelayUntil(&xLastWakeTime,xFrequency);
	}
  /* USER CODE END StartGoTask */
}
