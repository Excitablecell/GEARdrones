#include "main.h"
#include "math.h"
#include "cmsis_os.h"
#include "task.h"
#include "ssd1306.h"
#include "filter_task.h"
#include "cube.h"
#include <stdio.h>
#include "RTOSTask.h"
#include "Members.h"

extern Bar_Status BarStatus;
float Height_Line[128];

void Page_2(void const * argument){
	/* USER CODE BEGIN StartGoTask */
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20;
  xLastWakeTime = xTaskGetTickCount();
	uint8_t Height_p = 0;
	float Max_Height;
  /* Infinite loop */
  for(;;)
  {
		if(BarStatus.Page_Status == 2){
			if(BarStatus.Update_Flag == 1){BarStatus.Update_Flag = 0;SSD1306_Fill(SSD1306_COLOR_BLACK);SSD1306_UpdateScreen();}
			if(Height_p < 128){Height_p++;}else{Height_p = 0;SSD1306_Fill(SSD1306_COLOR_BLACK);}
			if(Height_Line[Height_p] > Max_Height){Max_Height = Height_Line[Height_p];}
			Height_Line[Height_p] = Height;
			if(Base_PWM != -1){
				if(Height_p > 0 && (Height_Line[Height_p-1]/7 < 65)){
					SSD1306_DrawLine(Height_p-1,(64-Height_Line[Height_p-1]/7),Height_p,(64-Height_Line[Height_p]/7), SSD1306_COLOR_WHITE);
				}
				SSD1306_GotoXY(0, 0); 
//				sprintf(Show_PWM, "Base: %d", Base_PWM);
				SSD1306_Puts(Show_PWM, &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_UpdateScreen();
			}
			
		}
		osDelayUntil(&xLastWakeTime,xFrequency);
	}
  /* USER CODE END StartGoTask */
}
