#include "vl53l0x_gen.h"
#include "cmsis_os.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK NANO STM32开发板
//VL53L0X-普通测量模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2018/7/18
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2018-2028
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

VL53L0X_RangingMeasurementData_t vl53l0x_data;//测距测量结构体
uint16_t Distance_data=0;//保存测距数据

//VL53L0X 测量模式配置
//dev:设备I2C参数结构体
//mode: 0:默认;1:高精度;2:长距离
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode)
{
	
	 VL53L0X_Error status = VL53L0X_ERROR_NONE;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	 vl53l0x_reset(dev);//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	 status = VL53L0X_StaticInit(dev);

   if(AjustOK!=0){
	    status= VL53L0X_SetReferenceSpads(dev,Vl53l0x_data.refSpadCount,Vl53l0x_data.isApertureSpads);//设定Spads校准值
      if(status!=VL53L0X_ERROR_NONE) goto error;	
      HAL_Delay(2);		 
	    status= VL53L0X_SetRefCalibration(dev,Vl53l0x_data.VhvSettings,Vl53l0x_data.PhaseCal);//设定Ref校准值
		  if(status!=VL53L0X_ERROR_NONE) goto error;
		  HAL_Delay(2);
	    status=VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,Vl53l0x_data.OffsetMicroMeter);//设定偏移校准值
		  if(status!=VL53L0X_ERROR_NONE) goto error; 
		  HAL_Delay(2);
		  status=VL53L0X_SetXTalkCompensationRateMegaCps(dev,Vl53l0x_data.XTalkCompensationRateMegaCps);//设定串扰校准值
		  if(status!=VL53L0X_ERROR_NONE) goto error;
      HAL_Delay(2);
     }
	 else
	 {
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref参考校准
		if(status!=VL53L0X_ERROR_NONE) goto error;
		HAL_Delay(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//执行参考SPAD管理
		if(status!=VL53L0X_ERROR_NONE) goto error;
    HAL_Delay(2);		 	 
	 }
	 status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//使能连续测量模式
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 HAL_Delay(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//使能SIGMA范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 HAL_Delay(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//使能信号速率范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 HAL_Delay(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//设定SIGMA范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 HAL_Delay(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//设定信号速率范围范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 HAL_Delay(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//设定完整测距最长时间
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 HAL_Delay(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//设定VCSEL脉冲周期
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 HAL_Delay(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//设定VCSEL脉冲周期范围

	 VL53L0X_StartMeasurement(dev);//启动测量
	 
	 
	 error://错误信息
	 if(status!=VL53L0X_ERROR_NONE)
	 {
		return status;
	 }
	 return status;
	
}	

VL53L0X_Error vl53l0x_interrupt_start(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf)
{
	VL53L0X_Error status=VL53L0X_ERROR_NONE;//工作状态
	
	uint8_t RangeStatus;
	status = VL53L0X_GetRangingMeasurementData(dev,pdata);//获取测量距离,并且显示距离
	if(status !=VL53L0X_ERROR_NONE) return status;
	
	RangeStatus = pdata->RangeStatus;//获取当前测量状态
  memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus,buf);//根据测量状态读取状态字符串
	
	Distance_data = vl53l0x_data.RangeMilliMeter;//保存最近一次测距测量数据
	return status;
}

//VL53L0X 单次距离测量函数
//dev:设备I2C参数结构体
//pdata:保存测量数据结构体
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf)
{

	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;
	
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//执行单次测距并获取测距测量数据
	if(status !=VL53L0X_ERROR_NONE) return status;
   
	RangeStatus = pdata->RangeStatus;//获取当前测量状态
  memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus,buf);//根据测量状态读取状态字符串
	
	Distance_data = pdata->RangeMilliMeter;//保存最近一次测距测量数据

  return status;
}

//启动普通测量
//dev：设备I2C参数结构体
//mode模式配置 0:默认;1:高精度;2:长距离;3:高速
void vl53l0x_general_start(VL53L0X_Dev_t *dev,u8 mode)
{
	static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;//工作状态
	u8 i=0;
	
	vl53l0x_set_mode(dev,mode);//配置测量模式
	while(1)
	{
		 if(Status==VL53L0X_ERROR_NONE)
		 {
				Status = vl53l0x_start_single_test(dev,&vl53l0x_data,buf);//执行一次测量
				//printf("State;%i , %s\r\n",vl53l0x_data.RangeStatus,buf);//打印测量状态	
				//printf("d: %4imm\r\n",Distance_data);//打印测量距离
		 }
		 i++; 
		 if(i==5)
		 {
			 i=0;
		 }
         osDelay(50);	
		
	}	
	
}

//vl53l0x普通测量模式测试
//dev:设备I2C参数结构体
//mode:测量模式
void vl53l0x_general_test(VL53L0X_Dev_t *dev, u8 mode)
{ 
	u8 i=0;
	while(1)
	{	

		if(1)//启动测量
		{
			vl53l0x_general_start(dev,mode);//启动测量
			
		}			
		i++;
		if(i==5)
		{
			i=0;
		}
		HAL_Delay(50);

	}
	
}
