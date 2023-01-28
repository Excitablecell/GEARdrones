#include "main.h"
#include "math.h"
#include "cmsis_os.h"
#include "task.h"

#include "RTOSTask.h"
#include "ssd1306.h"
#include "filter_task.h"
#include "filter.h"
#include "cube.h"
#include <stdio.h>
#include "Members.h"
#include "UWB.h"

MeanFilter<100> Power;
extern UWB Uwb;
extern Bar_Status BarStatus;
char Show_VBAT[10],Show_Angle[2] = {0,0},Show_PWM[2];
float x_speed,y_speed,Height;
int16_t Base_PWM;
uint8_t vbat_show;
void Main_Page(void const * argument){
	/* USER CODE BEGIN StartGoTask */
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
		osDelayUntil(&xLastWakeTime,xFrequency);
	}
  /* USER CODE END StartGoTask */
}
