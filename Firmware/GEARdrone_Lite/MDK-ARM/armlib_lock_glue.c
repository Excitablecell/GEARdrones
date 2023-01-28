/**
  ******************************************************************************
  * @file      armlib_lock_glue.c
  * @author    STMicroelectronics
  * @brief     Implementation of ARM C library lock interface
  *
  * @details   For more information about which C functions
  *            need which of these lowlevel functions
  *            please consult the "Arm C and C++ Libraries and
  *            Floating-Point Support User Guide"
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#if !defined(__CC_ARM) && !(defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
#error "armlib_lock_glue.c" should be used with ARM Compilers only
#endif /* !defined(__CC_ARM) && !(defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) */

/* Includes ------------------------------------------------------------------*/
#include <cmsis_compiler.h>

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Global Error_Handler
  */
__WEAK void Error_Handler(void)
{
  /* Not used if it exists in project */
  while (1);
}

#ifdef __MICROLIB
#warning Microlib does not provide mutex locks to guard against code that is not thread safe
#else

/* Includes ------------------------------------------------------------------*/
#include "stm32_lock.h"

/* Private typedef -----------------------------------------------------------*/
typedef void *mutex_t;

struct __lock
{
  uint8_t initialized; /**< Flag to indicate that lock is initialized */
  LockingData_t lock_data; /**< The locking data */
};

/* Private defines -----------------------------------------------------------*/
/** Maximal number of static allocated locks */
#define MAX_LOCK 8

/* Private macros ------------------------------------------------------------*/
/** Convert pointer to pointer to instance of struct __lock */
#define STM32_GET_LOCK_PTR(mutex_ptr) ((struct __lock *) *(mutex_ptr))

/** See struct __lock definition */
#define STM32_LOCK_PARAMETER(lock_ptr) (&(lock_ptr)->lock_data)

/** See struct __lock definition */
#define STM32_LOCK_INITIALIZED(lock_ptr) ((lock_ptr)->initialized)

/* Private variables ---------------------------------------------------------*/
/** Maximum system locks allowed by armlib */
static struct __lock static_lock[MAX_LOCK];

/** Lock for static_lock array */
static LockingData_t static_list_lock = LOCKING_DATA_INIT;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Thread safety Initialization, called before main
  */
__attribute__ ((constructor)) void __armlib_thread_safety_init()
{
  uint32_t index;

  /* Mark all system locks as not initialized */
  stm32_lock_acquire(&static_list_lock);
  for (index = 0; index < STM32_LOCK_ARRAY_SIZE(static_lock); index++)
  {
    STM32_LOCK_INITIALIZED(&static_lock[index]) = 0;
  }
  stm32_lock_release(&static_list_lock);
}

/**
  * @defgroup _mutex_functions ARM library locks
  * @{
  */

/**
  * @brief Initialize lock mutex
  * @param lock The lock
  * @return 0 on failure
  */
__attribute__((used)) int _mutex_initialize(mutex_t *lock)
{
  if (lock != NULL)
  {
    uint32_t index;

    stm32_lock_acquire(&static_list_lock);
    for (index = 0; index < STM32_LOCK_ARRAY_SIZE(static_lock); index++)
    {
      if (STM32_LOCK_INITIALIZED(&static_lock[index]) == 0)
      {
        *lock = &static_lock[index];
        STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) = 1;
        stm32_lock_init(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
        stm32_lock_release(&static_list_lock);
        return 1;
      }
    }
    stm32_lock_release(&static_list_lock);
  }

  /* Not enough mutexes, MAX_LOCK should be incremented */
  return 0;
}

/**
  * @brief Acquire lock mutex
  * @param lock The lock
  */
__attribute__((used)) void _mutex_acquire(mutex_t *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    stm32_lock_acquire(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_list_lock);
}

/**
  * @brief Release lock mutex
  * @param lock The lock
  */
__attribute__((used)) void _mutex_release(mutex_t *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    stm32_lock_release(STM32_LOCK_PARAMETER(STM32_GET_LOCK_PTR(lock)));
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_list_lock);
}

/**
  * @brief Free lock mutex
  * @param lock The lock
  */
__attribute__((used)) void _mutex_free(mutex_t *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);

  stm32_lock_acquire(&static_list_lock);
  if (STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) == 1)
  {
    STM32_LOCK_INITIALIZED(STM32_GET_LOCK_PTR(lock)) = 0;
  }
  else
  {
    STM32_LOCK_BLOCK();
  }
  stm32_lock_release(&static_list_lock);
}

/**
  * @}
  */

#endif /* __MICROLIB */
