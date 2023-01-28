/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    NLR_localization.cpp
  * @author  EXcai
  * @brief   UWB 基于非线性回归分析的无人机初始化定位(5机)
  *
  ==============================================================================
													How to use this library 怎么使用这个库？
  ==============================================================================
    @note
			- "An Ultra-Wideband-based Multi-UAV Localization System in GPS-denied environments"
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
#include "NLR_localization.h"
#include "Drone.h"

/* Private variables ---------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/********************************************************************************
 * @brief  执行一次估计 
 * @param  	输入最新uwb测量距离distance[10]
 * @retval 	输出本次未知坐标的估计unk[6]
 * @attention 
							-该函数用于5机编队的初始化位置估计，起飞前需要在函数外实时传入数据不停估计，五架飞机的编号必须为0-4！！
							-该函数以第0 第1 第2架为锚点，必须标号为0的飞机放置在原点处，标号为1的飞机精确放置在(0,1m)处，标号为2的飞机精确放置在(0,1m)处
							-5架飞机排列方式如下：	
									标号为0的飞机放置在原点处，标号为1的飞机精确放置在(1m,0)处，标号为2的飞机精确放置在(0,1m)处
									标号为3的飞机放置在(1,1m)处(大概位置)
									标号为4的飞机放置在(0.5m,0.5m)处(大概位置)
									UAV2       	UAV3
												UAV4
									UAV0				UAV1
									
			
 * @author EXcai(亢体造梦)
 *******************************************************************************/
float NLR::Nlr_Estimate(float duwb[7]){
	
//	for(int i=0;i<7;i++){
//		if(duwb[i] == 0){
//			error_step = 255;
//			return error_step;//测距数组不完整
//		}
//	}
	
	/* Cols1 -------------------------------------------------------*/
	JK[0] = (unk[0]-known[0])/__sqrtf(powf((unk[0]-known[0]),2) + powf((unk[1]-known[1]),2));
	JK[1] = (unk[1]-known[1])/__sqrtf(powf((unk[0]-known[0]),2) + powf((unk[1]-known[1]),2));
	/* Cols2 -------------------------------------------------------*/
	JK[6] = (unk[2]-known[0])/__sqrtf(powf((unk[2]-known[0]),2) + powf((unk[3]-known[1]),2));
	JK[7] = (unk[3]-known[1])/__sqrtf(powf((unk[2]-known[0]),2) + powf((unk[3]-known[1]),2));
	/* Cols3 -------------------------------------------------------*/
	JK[8] = (unk[0]-known[2])/__sqrtf(powf((unk[0]-known[2]),2) + powf((unk[1]-known[3]),2));
	JK[9] = (unk[1]-known[3])/__sqrtf(powf((unk[0]-known[2]),2) + powf((unk[1]-known[3]),2));
	/* Cols4 -------------------------------------------------------*/
	JK[14] = (unk[2]-known[2])/__sqrtf(powf((unk[2]-known[2]),2) + powf((unk[3]-known[3]),2));
	JK[15] = (unk[3]-known[3])/__sqrtf(powf((unk[2]-known[2]),2) + powf((unk[3]-known[3]),2));
	/* Cols5 -------------------------------------------------------*/
	JK[16] = (unk[0]-known[4])/__sqrtf(powf((unk[0]-known[4]),2) + powf((unk[1]-known[5]),2));
	JK[17] = (unk[1]-known[5])/__sqrtf(powf((unk[0]-known[4]),2) + powf((unk[1]-known[5]),2));
	/* Cols6 -------------------------------------------------------*/
	JK[22] = (unk[2]-known[4])/__sqrtf(powf((unk[2]-known[4]),2) + powf((unk[3]-known[5]),2));
	JK[23] = (unk[3]-known[5])/__sqrtf(powf((unk[2]-known[4]),2) + powf((unk[3]-known[5]),2));
	/* Cols7 -------------------------------------------------------*/
	JK[24] = (unk[0]-unk[2])/__sqrtf(powf(unk[0]-unk[2],2) + powf(unk[1]-unk[3],2));
	JK[25] = (unk[1]-unk[3])/__sqrtf(powf(unk[0]-unk[2],2) + powf(unk[1]-unk[3],2));
	JK[26] = (unk[2]-unk[0])/__sqrtf(powf(unk[2]-unk[0],2) + powf(unk[3]-unk[1],2));
	JK[27] = (unk[3]-unk[1])/__sqrtf(powf(unk[2]-unk[0],2) + powf(unk[3]-unk[1],2));
	
	jk_mat.numCols = 4;//列
	jk_mat.numRows = 7;//行
	jk_mat.pData = JK;
	
	jk_trans_mat.numCols = 7;
	jk_trans_mat.numRows = 4;
	jk_trans_mat.pData = jk_trans_temp;
	error_step = arm_mat_trans_f32(&jk_mat,&jk_trans_mat);//求转置
	if(error_step != 0){error_step = 10-error_step;return error_step;}
	
	jk_multi1.numCols = 4;
	jk_multi1.numRows = 4;
	jk_multi1.pData = jk_multi_temp1;
	error_step = arm_mat_mult_f32(&jk_trans_mat,&jk_mat,&jk_multi1);//矩阵乘法
	if(error_step != 0){error_step = 20-error_step;return error_step;}
	
	jk_inverse.numCols = 4;
	jk_inverse.numRows = 4;
	jk_inverse.pData = jk_inverse_temp;
	error_step = arm_mat_inverse_f32(&jk_multi1,&jk_inverse);//求逆
	if(error_step != 0){error_step = 30-error_step;return error_step;}
	
	jk_final.numCols = 7;
	jk_final.numRows = 4;
	jk_final.pData = jk_final_temp;
	error_step = arm_mat_mult_f32(&jk_inverse,&jk_trans_mat,&jk_final);//求最终变换的JK结果
	if(error_step != 0){error_step = 40-error_step;return error_step;}
	 
	jk_scale.numCols = 7;
	jk_scale.numRows = 4;
	jk_scale.pData = jk_scale_temp;
	error_step = arm_mat_scale_f32(&jk_final,0.5f,&jk_scale);//限制步长为0.5f
	if(error_step != 0){error_step = 50-error_step;return error_step;}
	
	/* 获得deltaD = 本次测量距离 - 本次估计距离*/
	des[0] = duwb[0] - __sqrtf(powf((unk[0]-known[0]),2) + powf((unk[1]-known[1]),2));
	des[1] = duwb[1] - __sqrtf(powf((unk[2]-known[0]),2) + powf((unk[3]-known[1]),2));
	des[2] = duwb[2] - __sqrtf(powf((unk[0]-known[2]),2) + powf((unk[1]-known[3]),2));
	des[3] = duwb[3] - __sqrtf(powf((unk[2]-known[2]),2) + powf((unk[3]-known[3]),2));
	des[4] = duwb[4] - __sqrtf(powf((unk[0]-known[4]),2) + powf((unk[1]-known[5]),2));
	des[5] = duwb[5] - __sqrtf(powf((unk[2]-known[4]),2) + powf((unk[3]-known[5]),2));
	des[6] = duwb[6] - __sqrtf(powf(unk[0]-unk[2],2) + powf(unk[1]-unk[3],2));
	des_mat.numCols = 1;
	des_mat.numRows = 7;
	des_mat.pData = des;
	
	d_delta.numCols = 1;
	d_delta.numRows = 4;
	d_delta.pData = des_current;
	error_step = arm_mat_mult_f32(&jk_scale,&des_mat,&d_delta);//求delta_distance
	if(error_step != 0){error_step = 60-error_step;return error_step;}
	
	unk[0] = unk[0] + des_current[0];
	unk[1] = unk[1] + des_current[1];
	unk[2] = unk[2] + des_current[2];
	unk[3] = unk[3] + des_current[3];
	
	return 0;
}
