#include "main.h"
#include "math.h"
#include "cmsis_os.h"
#include "task.h"
#include "ssd1306.h"
#include "filter_task.h"
#include "filter.h"
#include "cube.h"
#include <stdio.h>
#include <string.h>
#include "RTOSTask.h"
#include "Members.h"
#include "UWB.h"

MeanFilter<4> LBar1,LBar2,RBar1,RBar2;
Bar_Status BarStatus;
UWB Uwb;

void User_Action(void const * argument){
	/* USER CODE BEGIN StartGoTask */
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();
	
//	printf("ATE0\r\n");
//	vTaskDelay(1500);
//	printf("AT+CWJAP=\"GEAR\",\"geardrone\"\r\n");
//	vTaskDelay(2000);
//	printf("AT+MQTTUSERCFG=0,1,\"ESP\",\"admin\",\"public\",0,0,\"\"\r\n");
//	vTaskDelay(200);
//	//printf("AT+MQTTCONN=0,\"192.168.0.139\",1883,0\r\n");
//	printf("AT+MQTTCONN=0,\"192.168.0.119\",1883,0\r\n");
//	vTaskDelay(1500);
	
  /* Infinite loop */
  for(;;)
  {
		Uwb.User_Unpack();
		//ËÀÇø
		if(volt[0] < 2200 && volt[0] > 1500){rate[0] = 5;}else{rate[0] = (volt[0]/40)/10;}
		if(volt[1] < 2200 && volt[1] > 1650){rate[1] = 5;}else{rate[1] = (102-volt[1]/40)/10;}
		rate[2] = (102-volt[2]/40)/10;
		if(volt[3] < 2200 && volt[3] > 1500){rate[3] = 5;}else{rate[3] = (102-volt[3]/40)/10;}
		if(rate[0] > 9){rate[0] = 9;}
		if(rate[1] > 9){rate[1] = 9;}
		if(rate[2] > 9){rate[2] = 9;}
		if(rate[3] > 9){rate[3] = 9;}
		if(rate[2] > 1){HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);}else{HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);}
//		printf("AT+MQTTPUB=0,\"Remote\",\"[%d%d%d%d]\",1,0\r\n",rate[0],rate[1],rate[2],rate[3]);
		memcpy(&rate[4],&Uwb.others_data[1].data,12);
		memcpy(&rate[16],&Uwb.others_data[2].data,12);
		memcpy(&rate[28],&Uwb.others_data[3].data,12);
		memcpy(&rate[40],&Uwb.others_data[4].data,12);
		HAL_UART_Transmit(&huart3,rate,sizeof(rate),0xff);
			
		osDelayUntil(&xLastWakeTime,xFrequency);
	}
  /* USER CODE END StartGoTask */
}

