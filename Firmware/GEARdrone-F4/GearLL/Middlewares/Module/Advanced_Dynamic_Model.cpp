/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    Drone.cpp
  * @author  EXcai
  * @brief   无人机精细动力学模型
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
		- 参考
  	@warning
			- Standard C++11 required!

  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding
  * through your new brief.
  *

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Drone.h"

/* Private variables --------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/********************************************************************************
 * @brief  	无人机精细动力学模型

 * @param  	无人机精细动力学模型

 * @retval 	0: 成功
						1: 失败
 * @author	EXcai(亢体造梦)
 *******************************************************************************/



uint8_t GearDrone::Advanced_Dynamic_Model() {

/*无人机精细动力学模型*/
    
    /*无人机物理参数*/
    float UAV_weight=0.0813;    //无人机重量（g）
    float UAV_arm=62.4265e-3;   //无人机臂长（m）
    float UAV_Ix=0.000226942f;//无人机三个方向转动惯量
    float UAV_Iy=0.000226942f;
    float UAV_Iz=0.000172605f;
    
    Pos_X_PID.Current=current_data.pos_x;
    Pos_X_PID.Target=target.pos_x_target;
    Pos_X_PID.Adjust();
    
    Pos_Y_PID.Current=current_data.pos_y;
    Pos_Y_PID.Target=target.pos_y_target;
    Pos_Y_PID.Adjust();
    
    Height_PID.Current=current_data.pos_z;
    Height_PID.Target=target.pos_z_target;
    Height_PID.Adjust();
    
    //U2_1=arm*(F2-F4)
    
//       Pitch_PID.Target=(Pos_Y_PID.Out*sin(current_data.yaw*3.14159f/180)-Pos_X_PID.Out*cos(current_data.yaw*3.14159f/180))/9.81f;
			 Pitch_PID.Target = 0;
       Pitch_PID.Current=current_data.pitch;
			 Pitch_PID.Adjust();
       float u2_1=UAV_Iy*Pitch_PID.Out+(current_data.pitch_speed*3.14159f/180*UAV_Iz*current_data.yaw_speed*3.14159f/180
                                                - current_data.yaw_speed*3.14159f/180*UAV_Ix*current_data.pitch_speed*3.14159f/180);
     
    
    //U2_2=arm*(F3-F1)
    
//        Roll_PID.Target=(Pos_X_PID.Out*sin(current_data.yaw*3.14159f/180)+Pos_Y_PID.Out*cos(current_data.yaw*3.14159f/180))/9.81f;
				Pitch_PID.Target = 0;
       Roll_PID.Current=current_data.roll;
				Roll_PID.Adjust();
        float u2_2=Roll_PID.Out*UAV_Ix-(current_data.roll_speed*3.14159f/180*UAV_Iz*current_data.yaw_speed*3.14159f/180
                                                    -current_data.yaw_speed*3.14159f/180*UAV_Iy*current_data.roll_speed*3.14159f/180);   
     
    
     //U2_3=arm*(F1-F2+F3-F4)
    
        Yaw_PID.Target = 180;
        Yaw_PID.Current=current_data.yaw;
        Yaw_PID.Adjust();
        float u2_3= Yaw_PID.Out*UAV_Iz+(current_data.roll*3.14159f/180*UAV_Iy*current_data.pitch*3.14159f/180
                                            -current_data.pitch*3.14159f/180*UAV_Iy*current_data.roll*3.14159f/180);
     
    
     
     //U1=arm*(F1+F2+F3+F4)
     
         float u1=(Height_PID.Out-9.81)*UAV_weight;
     
     
     //左上F1,顺时针递增
     float F1=1000*(u1+u2_3-2*u2_2)/4/9.81f; //g
     float F2=1000*(u1-u2_3+2*u2_1)/4/9.81f;
     float F3=1000*(u1+u2_3+2*u2_2)/4/9.81f;
     float F4=1000*(u1-u2_3-2*u2_1)/4/9.81f;
     
        
/*曲线方程 */
    float V=current_data.VBAT;
    if(V>=4.05f)
    {
        LF_PWM=(120.5f*exp(0.02383f*F1)-110.0f)*100;
        LB_PWM=(120.5f*exp(0.02383f*F2)-110.0f)*100;
        RB_PWM=(120.5f*exp(0.02383f*F3)-110.0f)*100;
        RF_PWM=(120.5f*exp(0.02383f*F4)-110.0f)*100;
    }
        
    if(V<4.05f&&V>=4.00f)
    {
        LF_PWM=(138.7f*exp(0.0231f*F1)-127.4f)*100;
        LB_PWM=(138.7f*exp(0.0231f*F2)-127.4f)*100;
        RB_PWM=(138.7f*exp(0.0231f*F3)-127.4f)*100;
        RF_PWM=(138.7f*exp(0.0231f*F4)-127.4f)*100;
    }    
    
    if(V<4.00f&&V>=3.96f)
    {
        LF_PWM=(0.05332f*F1*F1+3.008f*F1+12.29f)*100;
        LB_PWM=(0.05332f*F2*F2+3.008f*F2+12.29f)*100;
        RB_PWM=(0.05332f*F3*F3+3.008f*F3+12.29f)*100;
        RF_PWM=(0.05332f*F4*F4+3.008f*F4+12.29f)*100;
    }

    if(V<3.96f&&V>=3.90f)
    {
        LF_PWM=(147.3f*exp(0.02261f*F1)-133.7f)*100;
        LB_PWM=(147.3f*exp(0.02261f*F2)-133.7f)*100;
        RB_PWM=(147.3f*exp(0.02261f*F3)-133.7f)*100;
        RF_PWM=(147.3f*exp(0.02261f*F4)-133.7f)*100;
    }

    if(V<3.90f&&V>=3.84f)
    {
        LF_PWM=(0.06581f*F1*F1+3.013f*F1+14.88f)*100;
        LB_PWM=(0.06581f*F2*F2+3.013f*F2+14.88f)*100;
        RB_PWM=(0.06581f*F3*F3+3.013f*F3+14.88f)*100;
        RF_PWM=(0.06581f*F4*F4+3.013f*F4+14.88f)*100;
    }

    if(V<3.84f)
    {
        LF_PWM=(0.06621f*F1*F1+2.451f*F1+20.53f)*100;
        LB_PWM=(0.06621f*F2*F2+2.451f*F2+20.53f)*100;
        RB_PWM=(0.06621f*F3*F3+2.451f*F3+20.53f)*100;
        RF_PWM=(0.06621f*F4*F4+2.451f*F4+20.53f)*100;
    }

    
        
    /*PWM占空比上下限幅*/
	if(LF_PWM > 20000){LF_PWM = 20000;}
	if(RF_PWM > 20000){RF_PWM = 20000;}
	if(LB_PWM > 20000){LB_PWM = 20000;}
	if(RB_PWM > 20000){RB_PWM = 20000;}
	if(LF_PWM <= 0){LF_PWM = 0;}
	if(RF_PWM <= 0){RF_PWM = 0;}
	if(LB_PWM <= 0){LB_PWM = 0;}
	if(RB_PWM <= 0){RB_PWM = 0;}
     
    return 0;
}




