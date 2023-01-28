#include "main.h"
#include "cmsis_os.h"
#include "UpperMonitor.h"
#include "filter.h"
#include "tof_driver.h"
#include "task.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "PID.h"

#ifdef __cplusplus    
extern "C" {         
#endif
void Planning_Task(void const * argument);
#ifdef __cplusplus
}
#endif
extern uint16_t systemtime_10ms;
extern float Height,z_speed,x_speed,y_speed,Init_VBAT,sum_flow_x,sum_flow_y,mpu_vx,y_command,x_command,mpu_vy,target_x,target_y;
extern myPID Speed_x_PID, Speed_y_PID,Height_PID;

void Planning_Task(void const * argument)
{
  /* USER CODE BEGIN Upper_Monitor */
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 200;
  xLastWakeTime = xTaskGetTickCount();
	int Planning_Time;
  /* Infinite loop */
  for(;;)
  {
//		if(Height > 0.15 && systemtime_10ms > 1800){
//			target_x = 3*sinf(Planning_Time*0.1);
//			target_y = 3*cosf(Planning_Time*0.1);
//			Planning_Time++;
//		}
		osDelayUntil(&xLastWakeTime,xFrequency);
  }
  /* USER CODE END Upper_Monitor */
}
