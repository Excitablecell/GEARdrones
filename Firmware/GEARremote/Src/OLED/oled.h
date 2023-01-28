#ifndef __OLED_H
#define __OLED_H	
 
  	 
#include "main.h"	
 
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
 
#define OLED_MODE 0
 
/****************时钟*********************/
#define OLED_SCLK_PORT  (GPIOB)
#define OLED_SCLK_PINS  (GPIO_PIN_6)
 
/****************数据*********************/
#define OLED_SDIN_PORT  (GPIOB)
#define OLED_SDIN_PINS  (GPIO_PIN_7)
 
 
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(OLED_SCLK_PORT,OLED_SCLK_PINS,GPIO_PIN_RESET)
#define OLED_SCLK_Set() HAL_GPIO_WritePin(OLED_SCLK_PORT,OLED_SCLK_PINS,GPIO_PIN_SET)
 
#define OLED_SDIN_Clr() HAL_GPIO_WritePin(OLED_SDIN_PORT,OLED_SDIN_PINS,GPIO_PIN_RESET)
#define OLED_SDIN_Set() HAL_GPIO_WritePin(OLED_SDIN_PORT,OLED_SDIN_PINS,GPIO_PIN_SET)
 
 
 
//OLED模式设置
//0:4线串行模式
//1:并行8080模式
 
#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED端口定义----------------  					   
	     
 
//OLED控制用函数
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size2);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void LED_ON(void);
void LED_OFF(void);
#endif  
