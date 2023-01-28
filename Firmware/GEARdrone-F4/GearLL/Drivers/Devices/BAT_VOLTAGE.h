/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    BAT_VOLTAGE.h
  * @author  EXcai
  * @brief   电池电压监测
  *
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *

  *******************************************************************************/


#ifndef __BAT_VOLTAGE_H_
#define __BAT_VOLTAGE_H_

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "Sensors.h"
#include "filter.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Variables -------------------------------------------------------------------*/
	
struct adc_data{
	
	bool Status;
	uint32_t timestamp_ms;
	
	float BAT_Voltage,Chip_Temperature;
};
	
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart1;
	
#ifdef __cplusplus
}
#endif

/** 
* @brief Class for Voltage_ADC.
*/
class Voltage_ADC : public Sensor<adc_data>{
public:
	Voltage_ADC(){};
	virtual ~Voltage_ADC(){};

	virtual uint8_t Init(){
		return HAL_ADC_Start_DMA(&hadc1,volt,3);
	}
	
	virtual uint8_t Calibration(){
		if(VREFINT_CAL == 0 || TS_CAL1 == 0 || TS_CAL2 == 0){
			VREFINT_CAL = *(__IO uint16_t *) 0x1FFF7A2A; //获取校准值
			TS_CAL1 = *(__IO uint16_t *)(0x1FFF7A2C);
			TS_CAL2 = *(__IO uint16_t *)(0x1FFF7A2E);
		}
		return 0;
	}
	virtual bool Calibration_Status(){if(VREFINT_CAL == 0 || TS_CAL1 == 0 || TS_CAL2 == 0){return 1;}else return 0;}
	virtual struct adc_data Get_Value(){
		
		adc_data data;
		
		Power << (3.3f*VREFINT_CAL/volt[1])*volt[0]/2048+0.09f;
		Power >> BAT_Voltage;
		Chip_Temperature = ((110.0f - 30.0f) / (TS_CAL2 - TS_CAL1)) * (volt[2] - TS_CAL1) + 30.0f;
		
		data.timestamp_ms = System_time_us/1000;
		data.BAT_Voltage = BAT_Voltage;
		data.Chip_Temperature = Chip_Temperature;
		
		return data;
	}
	uint8_t Status;
	
#ifndef USE_UPPERMONITOR
	private:
#endif
	uint32_t volt[3];
	float BAT_Voltage,Chip_Temperature;
	MeanFilter<10> Power;
	uint16_t VREFINT_CAL,TS_CAL1,TS_CAL2;
	
};

#endif
