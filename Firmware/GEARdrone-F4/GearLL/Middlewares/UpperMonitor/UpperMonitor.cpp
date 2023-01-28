/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    UpperMonitor.c
  * @author  LiangHong Lin(林亮洪) 
  * @brief   Code for Upper monitor supported by Mr.Lin in STM32F4.
  * @date    Unkown.
  * @version 1.0
  * @par Change Log：
  * <table
  * <tr><th>Date        <th>Version  <th>Author    		  <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>LiangHong Lin  <td>Creator
  * </table>2019-11-06  <td> 1.1     <td>Mentos Seetoo  <td>Add return valie for 
  *                                                         `RecHandle()`
  *
  ==============================================================================
                      ##### How to use this driver #####
  ==============================================================================
    @note
      -# 在下面指定区域`extern`需要观察或者修改的变量。 \n
      -# 如果要观察变量，按格式改`UpperMonitor_Sent_Choose()`或 \n
         如果要修改变量，按格式改`PARAMETER_MODIFICATION()`。
      -# 调用`RecHandle()`处理上位机发过来的所有数据包(通常在串口中断函数中直接调用)
      -# 调用`Sent_Contorl()`发送数据给上位机。
    @warning
      -# 用户需要根据硬件连接修改`Sent_Contorl()`里用于串口发送的函数。

  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
	
/***********************上位机调参使用***********************/
/* 在这里extern需要使用的变量和需要包含的头文件 */


/***********************上位机调参使用***********************/

/* Includes ------------------------------------------------------------------*/ 
#include "UpperMonitor.h"
#include "PID.h"
#include "System_Startup.h"
#include "Drone.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern GearDrone Drone;

/** 
* @brief 千万不要改以下变量内容！！！！
*/
#define Sent_Data_Num 9
uint8_t On_Off_flag;
type_change Sent_data_type[Sent_Data_Num+2];              //传输数据共用体
uint8_t USART0_Sent_Choose_Data[9]={0,1,2,3,4,5,6,7,8};   //串口选择发送的数据标志

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void UpperMonitor_Sent_Set(float *data);
void UpperMonitor_Sent_Choose(float * data);
float PARAMETER_Change_float(uint8_t * PARAMETER);
void PARAMETER_MODIFICATION(uint8_t * PARAMETER);
void MODE_MODIFICATION(uint8_t * PARAMETER);
/* function prototypes -------------------------------------------------------*/
/**
* @brief  发送数据函数
* @param  None
* @return None
*/
void Sent_Contorl(UART_HandleTypeDef* huart_x)
{
  float temp[Sent_Data_Num];
  UpperMonitor_Sent_Choose(temp);                         //选择要传输的数据
  UpperMonitor_Sent_Set(temp);                            //发送数据转换格式
  HAL_UART_Transmit_DMA(huart_x,(uint8_t*)Sent_data_type+3,39);
}

/**
* @brief  串口发送参数选择函数(要观看的曲线),用于选择需要传输的数据
* @param  data:需要传输的数组指针
* @return None.
*/

void UpperMonitor_Sent_Choose(float * data)
{
  uint8_t i;
  for(i=0;i<Sent_Data_Num;i++)
  {
    switch(USART0_Sent_Choose_Data[i])
    {
#ifdef USE_UPPERMONITOR

      /* 以下部分用于观察参数曲线 */
//			case 0: data[i] = Drone.current_data.yaw;
//          break;
//			case 1: data[i] = Drone.target.yaw_target;
//          break;
//			case 2: data[i] = Drone.current_data.yaw_speed;
//          break;
//			case 3: data[i] = Drone.Yaw_PID.Out;
//          break;
//			case 4: data[i] = Drone.Yaw_Speed_PID.Out;
//          break;
//			case 5: data[i] = Drone.current_data.VBAT;
//          break;
			case 0: data[i] = Drone.current_data.x_speed;
          break;
			case 1: data[i] = Drone.current_data.y_speed;
          break;
			case 2: data[i] = Drone.roll_speed_wait;
          break;
			case 3: data[i] = Drone.pitch_speed_wait;
          break;
      default:break;
			
#endif
	  /* 以上部分用于观察参数曲线 */
    }
  }
}

/**
* @brief  上位机参数修改函数（要调的参数）
* @param  PARAMETER：指令数组指针，用于读取指令
* @return None.
*/

void PARAMETER_MODIFICATION(uint8_t * PARAMETER)
{
  switch(PARAMETER[0])
  {
    /* 以下部分用于修改参数内容 */
//    case 0x00: kp =PARAMETER_Change_float(PARAMETER+1);
//          break;
//		case 0x01: kd =PARAMETER_Change_float(PARAMETER+1);
//          break;
//		case 0x02: pitch_target =PARAMETER_Change_float(PARAMETER+1);
//          break;
//	  case 0x03: Base_PWM =PARAMETER_Change_float(PARAMETER+1);
//          break;
//		case 0x04: Middle = PARAMETER_Change_float(PARAMETER+1);
//					break;
//		case 0x05: PID_Velocity.Target = PARAMETER_Change_float(PARAMETER+1);
//					break;
//		case 0x06: bluetooth = PARAMETER_Change_float(PARAMETER+1);
//					break;
//		case 0x07: K = PARAMETER_Change_float(PARAMETER+1);
//					break;
//		case 0x06: K = PARAMETER_Change_float(PARAMETER+1);
//					break;
//		case 0x00: Q_angle = PARAMETER_Change_float(PARAMETER+1);
//		break;
//		case 0x01: Q_gyro = PARAMETER_Change_float(PARAMETER+1);
//		break;
//		case 0x02: R_angle = PARAMETER_Change_float(PARAMETER+1);
//		break;
     default:break;
	/* 以上部分用于修改参数内容 */
  }
}

/**
* @brief  串口发送设置函数,用于设置DMA串口的数据
* @param  data:需要传输的数组指针
* @return None.
*/
void UpperMonitor_Sent_Set(float *data)
{
  uint8_t j;
  Sent_data_type[0].change_u8[3]=0xfd;                          //发送数据头
  for(j=1;j<Sent_Data_Num+1;j++)                                //数据体
  {
    Sent_data_type[j].change_float=data[j-1];
  }
  Sent_data_type[Sent_Data_Num+1].change_u8[0]=Sent_Data_Num;   //数据尾
  Sent_data_type[Sent_Data_Num+1].change_u8[1]=0xfe;            //校验位
}

/**
* @brief  上位机参数转变成浮点数函数
* @param  PARAMETER：指令数组指针，用于读取指令
* @return None.
*/
float PARAMETER_Change_float(uint8_t * PARAMETER)
{
  uint8_t i=0;
  union type_change Sent_data_temp;                       //传输数据共用体
  for(i=0;i<4;i++)
  {
    Sent_data_temp.change_u8[i]=PARAMETER[3-i];           //转换成共用体数据类型
  }
  return Sent_data_temp.change_float;                     //返回共用体转化后的数据
}



/**
* @brief  上位机参数修改函数
* @param  PARAMETER： 指令数组指针，用于读取指令
* @return None.
*/
void MODE_MODIFICATION(uint8_t * PARAMETER)
{
  switch(PARAMETER[0])
  {
    case 0x00: USART0_Sent_Choose_Data[0]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x01: USART0_Sent_Choose_Data[1]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x02: USART0_Sent_Choose_Data[2]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x03: USART0_Sent_Choose_Data[3]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x04: USART0_Sent_Choose_Data[4]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x05: USART0_Sent_Choose_Data[5]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x06: USART0_Sent_Choose_Data[6]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x07: USART0_Sent_Choose_Data[7]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x08: USART0_Sent_Choose_Data[8]=PARAMETER_Change_float(PARAMETER+1);
          break;
    default:break;
  }
}

uint8_t  USART_Interrupt_flag=0xff;           //串口中断标志位 
uint8_t  USART_Get_Num_Flag=0;                //串口数据获取标志
uint8_t  USART_receive[5]={0};                //串口接收缓存数组
int len=0;
/**
* @brief  串口接收解析函数
* @param  data_buf：接收到的数据指针
          length  ：数据长度
* @return No meaning.
*/
uint32_t RecHandle(uint8_t *data_buf,uint16_t length)
{
  uint8_t Temp=0;
    len=length;
  for(int i=0;i<length;i++)
  {
    Temp=data_buf[i]; 
    switch(USART_Interrupt_flag)
    {
      case 0xff:  //USART0_Interrupt_flag==0xff时为等待模式，等待指令头输入
            if(Temp==0xf0)                  //指令头，识别上位机发送了修改指令
              USART_Interrupt_flag=0xf0;    //下一个指令将进入模式选择模式
            break;
      case 0xf0:                            //进入模式选择
            if(Temp==0x00)                  //修改参数
            {
              USART_Interrupt_flag=0x00;    //进入参数修改模式
              USART_Get_Num_Flag=0;
            }
            else if(Temp==0x01)             //修改模式
            {
              USART_Interrupt_flag=0x01;    //进入模式修改模式
              USART_Get_Num_Flag=0;
            }
            else if(Temp==0x02)
            {
              USART_Interrupt_flag=0x02;    //进入模式修改模式
              USART_Get_Num_Flag=0;
            }
            break;
      case 0x00:
            USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //参数处理
            {
              PARAMETER_MODIFICATION(USART_receive);
              USART_Interrupt_flag=0xff;    //回到等待模式
            }
            break;
      case 0x01:
            USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //参数处理
            {
              MODE_MODIFICATION(USART_receive);
              USART_Interrupt_flag=0xff;    //回到等待模式
            }
            break;
      case 0x02:  USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //参数处理
            {
              if(USART_receive[0]==0x0a)
              {
                for(int j=1;j<5;j++)
                {
                  if(USART_receive[j]!=0x0a)
                    USART_Interrupt_flag=0xff;    //回到等待模式
                }
                if(USART_Interrupt_flag==0x02)
                {
                  On_Off_flag =1;
                  USART_Interrupt_flag=0xff;    //回到等待模式
                }
              }
              else if(USART_receive[0]==0xb0)
              {
                for(int j=1;j<5;j++)
                {
                  if(USART_receive[j]!=0xb0)
                    USART_Interrupt_flag=0xff;   //回到等待模式
                }
                if(USART_Interrupt_flag==0x02)
                {
                  On_Off_flag =0;
                  USART_Interrupt_flag=0xff;    //回到等待模式
                }
              }
              else 
                USART_Interrupt_flag=0xff;     //回到等待模式
            }
            break;
            
      default:  USART_Interrupt_flag=0xff;    //回到等待模式
            break;
    }
  }
  return 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

