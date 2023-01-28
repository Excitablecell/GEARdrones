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
#include "System_Startup.h"
#include "arm_math.h"

/* Private variables ---------------------------------------------------------*/

class NLR{
	public:
		NLR(float x0,float y0,float x1,float y1,float x2,float y2){
			known[0] = x0;
			known[1] = y0;
			known[2] = x1;
			known[3] = y1;
			known[4] = x2;
			known[5] = y2;
		}
		~NLR(){};
			
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
		float Nlr_Estimate(float duwb[7]);//
		float unk[4] = {2.4f,2.4f,1.2f,1.2f};//是个列向量，表示未知坐标的初始化估计：(x4,y4,x5,y5)' 无人机应当以边长为1的正方形排布，中间放置第5架飞机

			
	private:
			float error_step;//NLR估计错误：255为测距数组不完整，其他错误号十位数为出问题的步骤，个位数为步骤运算遇到的问题 ，具体查arm_math库对应的错误(比如说11：步骤1出错，错误号为1，查得为One or more arguments are incorrect)
			float known[6];//是个列向量，表示已知坐标：(x1,y1,x2,y2,x3,y3)' 必须标号为0的飞机放置在原点处，标号为1的飞机精确放置在(1m,0)处
	
			float des[7];//本次估计距离(des:distance_estimate)
			float JK[28];
			float estimate_error[7];//估计"误差"（与初值unk[]比）
			
			arm_matrix_instance_f32 jk_mat;
			arm_matrix_instance_f32 unk_mat;
			arm_matrix_instance_f32 des_mat;
			arm_matrix_instance_f32 jk_trans_mat;//求转置
			arm_matrix_instance_f32 jk_multi1;//矩阵乘法
			arm_matrix_instance_f32 jk_inverse;//求逆
			arm_matrix_instance_f32 jk_final;//求最终变换的JK结果
			arm_matrix_instance_f32 jk_scale;//限制步长
			arm_matrix_instance_f32 d_delta;//求delta_distance
			
			float32_t jk_trans_temp[28],jk_multi_temp1[16],jk_inverse_temp[16],jk_final_temp[28],jk_scale_temp[28],des_current[4];
};
