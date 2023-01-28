#ifndef _MEMBERS_H
#define _MEMBERS_H

#include "main.h"
#include "math.h"
#include "cmsis_os.h"
#include "task.h"

extern char Show_VBAT[10],Show_Angle[2],Show_PWM[2];
extern float x_speed,y_speed,Height;
extern int16_t Base_PWM;
extern uint8_t vbat_show;

extern UART_HandleTypeDef huart3;
extern uint8_t send_buffer[8];
extern uint8_t aRxBuffer1[400];		// husart3 rxbuff
extern unsigned int volt[6];
extern uint8_t rate[52];
extern float VBAT;
extern uint8_t button_flag,init_anime;

class Bar_Status
{
   public:
		float RBar1_Status,RBar2_Status,LBar1_Status,LBar2_Status;
		int8_t Page_Status,Update_Flag;	
};
#endif
