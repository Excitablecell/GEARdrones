/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    PID.cpp
  * @author  BigeYoung & M3chD09
  * @brief   PID controller sets. This file provides traditional PID controller 
  * @date    2019-06-26
  * @version 1.2
  *
  ==============================================================================
                            How to use this library
  ==============================================================================
    @note
      The following Controllers is provied by this lib: \n
      - Open loop controller.
      - Traditional PID controller.
    @warning
      - Function `getMicroTick_regist()` must be regist once before using this library!
      - 1 System tick in this library is standing for 1 us.
      - Header file `"filter.h"` required!
      - Standard C++11 required!
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
/* Includes ------------------------------------------------------------------*/
#include "PID.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SystemTick_Fun myPIDTimer::Get_SystemTick = NULL;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
template<typename Type> 
Type _pid_Abs(Type x) {return ((x > 0) ? x : -x);}

template<typename Type> 
void _pid_Constrain(Type *x, Type Min, Type Max) 
{
  if(*x < Min) *x = Min;
  else if(*x > Max) *x = Max;
  else{;}
}
/* function prototypes -------------------------------------------------------*/
uint8_t myPIDTimer::UpdataTimeStamp(void)
{
  uint32_t now_time;
  
  /*Check `Get_SystemTick` */
  if(myPIDTimer::Get_SystemTick != NULL)
  {
    /*Convert to system time*/
    if (last_time == 0)
    {
      last_time = myPIDTimer::Get_SystemTick();
      return 1;
    }
    now_time = myPIDTimer::Get_SystemTick();

    /*Overflow*/
    if (now_time < last_time)
      dt = (float)(now_time + (0xFFFFFFFF - last_time));
    else
      dt = (float)(now_time - last_time);

    last_time = now_time;

    dt *= 0.000001f;
    
    return 0;
  }
  else{
    dt = 0;
    return 1;
  }
}

/**
 * @brief  Regist get time function(1Tick = 1us)
 * @param  realTime_fun: Pointer of function to get system real time
 * @retval 1: success
           0: error input param
 * @author
 */
uint8_t myPIDTimer::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
  if(getTick_fun != NULL)
  {
    myPIDTimer::Get_SystemTick = getTick_fun;
    return 1;
  }
  else 
    return 0;
}

float myPID::Adjust()
{
  /*Error time*/
  if (UpdataTimeStamp())
    return 0;

//	dt = 0.05;
	
  Error = Target - Current;

  if (_pid_Abs(Error) < DeadZone)
  {
    Out = 0;
    return Out;
  }


  P_Term = Kp * Error;

  /* Using Integral separation */
  if (_pid_Abs(Error) < I_SeparThresh)
  {
    /* PID with Changing integration rate */
    float I_VarSpeedf = 0;
    if (_pid_Abs(Error) <= VarSpeed_I_B)
      I_VarSpeedf = 1;
    else if (_pid_Abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
      I_VarSpeedf = (VarSpeed_I_A - (_pid_Abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;
    
    if(Ki != 0){
      integral_e += I_VarSpeedf * Error * dt;
      /*Constrain*/
      _pid_Constrain(&integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
    }
    else{
      integral_e = 0;
    }
    
    I_Term = Ki * integral_e;
    /*Constarin*/
    _pid_Constrain(&integral_e, -I_Term_Max, I_Term_Max);
  }
  else{
    /*Clear*/
    I_Term = 0;
    integral_e = 0;
  }
  
  float d_err = 0;
  if (D_of_Current)
    d_err = (Current - pre_Current) / dt;
  else
    d_err = (Error - pre_error) / dt;
  D_Term = Kd * d_err;

  pre_error = Error;

  Out = P_Term + I_Term + D_Term;
  
  /* Constarin */
  _pid_Constrain(&Out, -Out_Max, Out_Max);
  
  return Out;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
