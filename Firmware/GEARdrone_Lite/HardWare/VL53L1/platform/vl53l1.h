#ifndef VL53L1_H
#define VL53L1_H
#include "vl53l1_api.h"
#include "main.h"
//detection mode
#define DEFAULT_MODE   0	//default,see manul 5.3.1
#define HIGH_ACCURACY  1
#define LONG_RANGE     2
#define HIGH_SPEED     3
#ifdef __cplusplus
 extern "C" {
#endif
//param struct for vl53l1x mode option, in manual 6.2 
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal,related to reflected amplitude 
	FixPoint1616_t sigmaLimit;     //Sigmal, related to distance mm
//	FixPoint1616_t ignoreThres;  //ignore threshold
	uint32_t timingBudget;         //When the ranging mode is set to timed ranging, user has to define the period of time
																 //between two consecutive measurements.
	uint8_t preRangeVcselPeriod ;  //VCSEL pulse cycle
	uint8_t finalRangeVcselPeriod ;//VCSEL pulse cycle period
}mode_data;


VL53L1_Error VL53L1Init(VL53L1_Dev_t* pDev);
VL53L1_Error VL53InitParam(VL53L1_Dev_t* pDev,uint8_t mode);
VL53L1_Error VL53Cali(VL53L1_Dev_t* pDev,void * save);
VL53L1_Error getDistance(VL53L1_Dev_t* pDev);

extern VL53L1_Dev_t VL53;
extern int32_t tof_distance_int32;
#ifdef __cplusplus
}
#endif
#endif
