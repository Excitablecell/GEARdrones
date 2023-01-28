#include "main.h"
#include "math.h"
#include "cmsis_os.h"
#include "task.h"

#include "RTOSTask.h"
#include "ssd1306.h"
#include "filter_task.h"
#include "cube.h"
#include <stdio.h>
#include "Members.h"
extern uint8_t config[];
extern uint8_t config2[];
extern Bar_Status BarStatus;

void Page_1(void const * argument){
	/* USER CODE BEGIN StartGoTask */
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
		if(BarStatus.Page_Status == 1){
			if(BarStatus.Update_Flag == 1){BarStatus.Update_Flag = 0;SSD1306_Fill(SSD1306_COLOR_BLACK);SSD1306_UpdateScreen();}
			SSD1306_Fill(SSD1306_COLOR_BLACK);
			SSD1306_DrawFilledRectangle(108,rate[0]/2 + 6,9,5,SSD1306_COLOR_WHITE);//RFB画油门量
			SSD1306_DrawRectangle(110,10,5,50,SSD1306_COLOR_WHITE);
			SSD1306_DrawFilledRectangle(118,rate[1]/2 + 6,9,5,SSD1306_COLOR_WHITE);//RLR画油门量
			SSD1306_DrawRectangle(120,10,5,50,SSD1306_COLOR_WHITE);
			SSD1306_DrawFilledRectangle(2,rate[2]/2 + 6,9,5,SSD1306_COLOR_WHITE);//LFB画油门量
			SSD1306_DrawRectangle(4,10,5,50,SSD1306_COLOR_WHITE);
			SSD1306_DrawFilledRectangle(12,rate[3]/2 + 6,9,5,SSD1306_COLOR_WHITE);//LLR画油门量
			SSD1306_DrawRectangle(14,10,5,50,SSD1306_COLOR_WHITE);
			SSD1306_DrawFilledRectangle(12,rate[3]/2 + 6,9,5,SSD1306_COLOR_WHITE);//LLR画油门量
			SSD1306_DrawRectangle(14,10,5,50,SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
		}
		osDelayUntil(&xLastWakeTime,xFrequency);
	}
  /* USER CODE END StartGoTask */
}
