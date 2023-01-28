/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, SCUT-RobotLab Development Team && EXcai(亢体动画)
  * @file    filter.h
  * @author  buff EXcai
  * @brief   Filter set in general signal process and analysis.
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
#ifndef _FILTER_H
#define _FILTER_H

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
#include <algorithm>
#include <math.h>
#include <string.h>
/* Exported function declarations --------------------------------------------*/
/* LowPassFilter */

class LowPassFilter
{
  public:
  /**
    @brief trust (0,1) 
   */
	LowPassFilter(float trust = 1): Trust(trust)
  {
   	now_num = last_num = 0;
  } 
  	~LowPassFilter(){};
    float Trust;
    void operator<< (const float& );
    void operator>> (float& );
    float f(float num);
  protected:
  	void in(float num);
  	float out();
  private:
    float now_num;
    float last_num;
};

/* MedianFilter	*/
template<int Length> 	
class MedianFilter
{
  /**
    @brief 滤波宽度(1,100)
   */
  public:
	MedianFilter()
	{
		static_assert((Length>0)&&(Length<101),"MedianFilter Length [1,100]");
		flag = Length;
		where_num = 0;
	} 						
  	~MedianFilter(){}; 
    void operator >> (float& num){ num = out();}
    void operator << (const float& num){in(num);}
    float f(float num)
    {
      in(num);
      return (out());
    }
  protected:
  	void in(float num)
    {
      now_num = num;
      /* flag=Length然后递减保证宽度内都是有效波值 */
      flag > 0? flag-- : 0;										
      buffer_num[where_num++] = num;
      where_num %= Length; 
    }
    
  	float out()
    {
      if(flag>0)
        return now_num;
      else
        {
          /* 准备排序 */
          memcpy(sort_num,buffer_num,sizeof(sort_num));	
          std::sort(sort_num,sort_num+Length);
          return sort_num[int(Length/2)];
        }
    }
    
  private:
  	float buffer_num[Length];
  	float sort_num[Length];
	float now_num;
	int flag,where_num;
};

/* MeanFilter */
template<int Length> 	
class MeanFilter
{
  public:
  /**
    @brief 滤波宽度(1,100)
   */
	MeanFilter()
	{
		static_assert((Length>0)&&(Length<101),"MedianFilter Length [1,100]");
		for(int x = 0 ; x < Length; x++) buffer_num[x] = 0;
		flag = Length;
		where_num = 0;
		sum = 0;
	} 						
  	~MeanFilter(){}; 
    void operator >> (float& num){ num = out();}	
    void operator << (const float& num){in(num);}
    float f(float num)
    {
      in(num);
      return (out());
    }
  protected:
  	void in(float num)
    {
      now_num = num;
      sum -= buffer_num[where_num];			  /*<! sum减去旧值 */
      sum += num;													/*<! sum加上新值 */
      buffer_num[where_num++] = num;
      flag > 0? flag-- : 0;								/*<!flag=Length然后递减保证宽度内都是有效波值 */
      where_num %= Length; 
    }
    
  	float out()
    {
      if(flag>0)
        return now_num;
      else
        return (sum/Length);
    }
  private:
  	float buffer_num[Length];
	float now_num;
	float sum; 						/*<! 宽度和数字和 */
	int flag,where_num;
};

/* ComplementaryFilter */
class ComplementaryFilter
{
  public:
  /** 
    @brief K 只能取0到1之间的小数
   */
	ComplementaryFilter(float _k): K(_k){}
  	~ComplementaryFilter(){};
    float K;
    float f(float Speed_in,float Acc_in){
      in(Speed_in,Acc_in);
	    return (out());
    };
  protected:
  	void in(float Speed_in,float Acc_in){
      acc = Acc_in;
      speed = Speed_in;
      speed_last = speed_mix;
    };
  	float out(){
      return speed_mix = (float)(K * speed + (1-K) * (speed_last + acc * 0.005f));
    };
  private:
    float speed;
    float acc;
    float speed_mix;
    float speed_last;
};

/* 二阶ComplementaryFilter */
class ComplementaryFilter2
{
  public:
  /** 
    @brief K 只能取0到1之间的小数
   */
	ComplementaryFilter2(float _k,float _dt){K = _k;dt = _dt;}
  	~ComplementaryFilter2(){};
    float f(float Speed_in,float Acc_in){
      in(Speed_in,Acc_in);
	    return (out());
    };
  protected:
  	void in(float Speed_in,float Acc_in){
      acc = Acc_in;
      speed = Speed_in;
      speed_last = speed_mix;
    };
  	float out(){
			y1 += (speed - speed_mix)*K*K*dt;
			speed_mix = speed_mix + y1 + 2 * K *(speed - speed_mix) + acc*dt;
      return speed_mix;
    };
  private:
		float K,dt;
		float y1;
    float speed;
    float acc;
    float speed_mix;
    float speed_last;
};

/* ButterworthFilter */
class ButterworthFilter
{
  public:
	  ButterworthFilter();
  	~ButterworthFilter(){};
		
    float f(float data){
      in(data);
	    return (out());
    };
    void operator<< (const float& data_in){
      in(data_in);
    };
    void operator>> (float& data_out){
      data_out = out();
    };
  protected:
  	void in(float data){
      Data = data;
    };
  	float out(){
      prod_1[0]= Data;
      res0_1[0] = (sos[0][0] * prod_1[0]) + (sos[0][1] * prod_1[1]) + (sos[0][2] * prod_1[2]) - (sos[0][4] * res0_1[1]) - (sos[0][5] * res0_1[2]);
      res_1[0] = (sos[1][0] * res0_1[0]) + (sos[1][1] * res0_1[1]) + (sos[1][2] * res0_1[2]) - (sos[1][4] * res_1[1]) - (sos[1][5] * res_1[2]);
      prod_1[2]=prod_1[1];
      prod_1[1]=prod_1[0];
      res0_1[2]=res0_1[1];
      res0_1[1]=res0_1[0];
      res_1[2]=res_1[1];
      res_1[1]=res_1[0];
      return res_1[0]; 
    };
  private:
    float Data;
    float b[2]={0.0000231,0.0001388};
    float a[2]={1,-4.5450};
    //滤波参数
    float sos[2][6]={{0.0219,0.1097,0.2194,0.2194,0.1097,0.0219},
    								 {1.0000,-0.9853,0.9738,-0.3864,0.1112,-0.0113}};
    float prod_1[3] = {0,0,0};
    float res0_1[3]= {0,0,0};
    float res_1[3]= {0,0,0};
};

/* KalmanFilter */
class KalmanFilter
{
  public:
  /** 
    @brief K 只能取0到1之间的小数
   */
		KalmanFilter(float _dt,float _speed_error,float _acc_error){
			dt = _dt;
			Q_mat[0] = _speed_error*_speed_error;
			Q_mat[3] = _acc_error*_acc_error;
			R_mat[0] = _speed_error*_speed_error;
			R_mat[3] = _acc_error*_acc_error;
		}
  	~KalmanFilter(){};
    float f(float Speed_in,float Acc_in){
      in(Speed_in,Acc_in);
	    return (out());
    };
  protected:
  	void in(float Speed_in,float Acc_in){
//			if(fabsf(Acc_in) > 0.01f){acc_ob = Acc_in;}else{acc_ob = 0;}
			acc_ob = Acc_in;
      speed_ob = Speed_in;
    };
  	float out(){
			
			speed_es = speed_es + dt*acc_ob;
//			acc_es = acc_ob;

			P_mat[0] = P_mat[0] + Q_mat[0] + (P_mat[1]+P_mat[2])*dt;
			P_mat[1] = P_mat[1] + P_mat[3]*dt;
			P_mat[2] = P_mat[2] + P_mat[3]*dt;
			P_mat[3] = P_mat[3] + Q_mat[3];
			
			K[0] = P_mat[0]/(P_mat[0] + R_mat[0]);
			K[1] = P_mat[2]/(P_mat[0] + R_mat[0]);
			
			speed_es = speed_es + K[0]*(speed_ob - speed_es);
			acc_es = acc_es + K[1]*(acc_ob - acc_es);
			
			P_mat[0] = P_mat[0]*(1-K[0]);
			P_mat[1] = P_mat[1]*(1-K[0]);
			P_mat[2] = P_mat[2]*(1-K[1]);
			P_mat[3] = P_mat[3]*(1-K[1]);
			
      return speed_es;
    };
  private:
		float dt;
		float P_mat[4] = {1,1,1,1},Q_mat[4],R_mat[4],K[2];
	float speed_es,speed_ob;//es:estimate;ob:observe
    float acc_es,acc_ob;
};

#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

