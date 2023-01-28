/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    filter.h
  * @author  buff
  * @brief   Filter set in general signal process and analysis.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _FILTER_H
#define _FILTER_H

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
#include <algorithm>
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
          return sort_num[int(Length)-1];
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



#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

