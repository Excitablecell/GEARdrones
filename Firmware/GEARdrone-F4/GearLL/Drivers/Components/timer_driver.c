/**
  ==============================================================================
                            How to use this driver  
  ==============================================================================
    @note
      -# 使用`Timer_Init()`设置延时定时器`TIM_X`,设置`delay_ms()`函数使用HAL库的实现
          `HAL_Delay()`或使用模块内方法实现。		
      -# 配置`TIM_X`自增时间为1us，在对应中断函数中加入:`Update_SystemTick();`。
      -# 在需要延时的地方使用`delay_us_nos()`或`delay_ms_noe()`。
		
    @warning
      -# 所有延时函数均为堵塞式延时。
      -# 使用前进行初始化
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timer_driver.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint32_t SystemTimerCnt;

struct timer_manage_obj_t
{
	TIM_HandleTypeDef*	htim_x;
}Timer_Manager;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
static void Error_Handler(void);

/* function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize Timer
* @param  htim_x : HAL Handler of timer x.
* @retval None
*/
void Timer_Init(TIM_HandleTypeDef* htim)
{
	/* Check the parameters */
	assert_param(htim != NULL);
	
	Timer_Manager.htim_x = htim;
    
  if(HAL_TIM_Base_Start_IT(Timer_Manager.htim_x)!=HAL_OK)
      Error_Handler();
}


/**
* @brief  Get the system tick from timer.
* @param  None
* @retval current tick.
*/
uint32_t Get_SystemTimer(void)
{
	return Timer_Manager.htim_x->Instance->CNT + SystemTimerCnt * 0xffffffff;
}

/**
* @brief  Update system tick that had run in Timer Interupt.
* @not    Add this function into Timer interupt function.
* @param  None
* @retval None
*/
void Update_SystemTick(void)
{
	SystemTimerCnt++;
}

/**
* @brief  Delay microsecond.
* @param  cnt : microsecond to delay 
* @retval None
*/
void delay_us_nos(uint32_t cnt)
{
	uint32_t temp = cnt  + microsecond();

	while(temp >= microsecond());
}

/**
* @brief  Delay millisecond.
* @param  cnt : millisecond to delay
* @retval None
*/
void delay_ms_nos(uint32_t cnt)
{
	if(Timer_Manager.htim_x != NULL)
	{
		uint32_t temp = cnt * 1000 + microsecond();
		while(temp >= microsecond());
	}
	else
		HAL_Delay(cnt);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
  /* Nromally the program would never run here. */
  while(1){}
}
