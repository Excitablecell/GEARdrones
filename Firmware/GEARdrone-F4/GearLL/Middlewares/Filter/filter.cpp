/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, SCUT-RobotLab Development Team && EXcai(亢体动画)
  * @file    filter.h
  * @author  buff EXcai
  * @brief   Filter set in general signal process and analysis.
  *
  ==============================================================================
													How to use this library 
  ==============================================================================
    @note
			- 低通滤波 
				-# LowPassFilter LF(trust);  
				-# 返回LF.f(num) 或者 LF << (in)  LF >> out 输出的时候才运算

			- 中值滤波 
				-# MedianFilter<Length> MDF;  
				-# 返回MDF.f(num) 或者 MDF << (in) MDF >> out 输出的时候才运算 

			- 均值滤波 
				-# MeanFilter<Length> MF;  
				-# 返回MF.f(num) 或者MF << (in)  MF >> out 输出的时候才运算 
      
			- 一阶互补滤波 
				-# ComplementaryFilter CF(K);  
				-# 返回CF.f(Speed_in,Acc_in) 输出的时候才运算 

			- 五阶巴特沃斯滤波 
				-# ButterworthFilter BWF;  
				-# 返回BWF.f(data) 或者BWF << (in)  BWF >> out 输出的时候才运算 

  	@warning 
			- 一阶互补滤波 K(0,1) 低通滤波部分trust (0,1) ！！！注意超过不报错   
			- 中值滤波 均值滤波(长度[1,100])
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
#include "filter.h"

/* Function prototypes -------------------------------------------------------*/
/* LowPassFilter */
void LowPassFilter::in(float num)							
{
	last_num = now_num;
	now_num = num;
}

float LowPassFilter::out()							
{
	return (now_num*Trust + last_num * (1 - Trust));
}


void LowPassFilter::operator <<(const float& num)			
{
	in(num);
}

void LowPassFilter::operator >>(float& num)
{
	num = out();
}

float LowPassFilter::f(float num)						
{
	in(num);
	return (out());
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
