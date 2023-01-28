#ifndef __BUTTERWORTH_H_
#define __BUTTERWORTH_H_

#include "main.h"
#include "cmsis_os.h"

/*********************************************************************************
		* @brief  巴特沃斯滤波器
		* @author EXcai亢体造梦
		********************************************************************************/

#define BLOCK_SIZE 1 /* 调用一次arm_biquad_cascade_df1_f32处理的采样点个数 */
static uint32_t blockSize = BLOCK_SIZE;
class ButterworthFilter{
	public:
		ButterworthFilter(){}
		~ButterworthFilter(){}
		
		/*********************************************************************************
		* @brief  巴特沃斯滤波器初始化
		* @param  params:matlab生成的滤波器系数，长度为order*5的float型数组，如：float IIRCoeffs32LP[5*order] = {1.0f, 2.0f, 1.0f, 1.143f,-0.413f};
			        scale：matlab生成的缩放系数
              order:2阶IIR滤波的个数,如6阶就为3
			
		* @author EXcai亢体造梦
		********************************************************************************/
		ButterworthFilter(float *params, float scale, uint8_t order){
			/* 巴特沃斯低通滤波器初始化 */
			IIRCoeffs32LP = params;
			ScaleValue = scale;
			arm_biquad_cascade_df1_init_f32(&S, order, (float *)&IIRCoeffs32LP[0], (float *)&IIRStateF32[0]);
		} 
		
		/*********************************************************************************
		* @brief  滤波函数
		* @param  input：需要背滤波的原始数据
			
		* @author EXcai亢体造梦
		********************************************************************************/
		float f(float input){
			input_data = input;
			arm_biquad_cascade_df1_f32(&S, &input_data,&output_data,blockSize);
			return output_data*ScaleValue;
		}
	
	private:
		float input_data,output_data;
		float ScaleValue;
		float IIRStateF32[4]; /* 状态缓存 */
		float *IIRCoeffs32LP; /* 巴特沃斯低通滤波器系数*/ 
		arm_biquad_casd_df1_inst_f32 S;
};

/*********************************************************************************
		* @brief  互补滤波器
		* @author EXcai亢体造梦
		********************************************************************************/

class ComplementaryFilter
{
	public:
		ComplementaryFilter(float k): K(k){} 
		float f(float value, float d_value,float timespan){
				result = K * value + (1-K) * (last_value + d_value * timespan);
				last_value = result;
				return result;
		}
	protected:
		float K,last_value,result;
};

#endif
