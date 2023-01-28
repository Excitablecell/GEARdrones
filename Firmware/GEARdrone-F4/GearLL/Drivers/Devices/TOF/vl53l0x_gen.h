#ifndef __VL53L0X_GEN_H
#define __VL53L0X_GEN_H

#define u8 uint8_t 
#include "vl53l0x.h"
#ifdef __cplusplus    
extern "C" {         
#endif
	
void vl53l0x_general_test(VL53L0X_Dev_t *dev,u8 mode);
void vl53l0x_general_start(VL53L0X_Dev_t *dev,u8 mode);
VL53L0X_Error vl53l0x_interrupt_start(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf);
	
#ifdef __cplusplus
}
#endif
#endif


