/**
 *******************************************************************************
  * Copyright (c) 2022 - ~, SCUT-RobotLab Development Team && EXcai(亢体动画)
  * @file    filter.h
  * @author  SCUT-RobotLab Development Team && EXcai
  * @brief   flash
  *
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */
#ifndef _FLASH_DRIVER_H_
#define _FLASH_DRIVER_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base address of Sector 7, 128 Kbytes  */
#define FLASH_END_ADDR      ((uint32_t)0x08080000)

/* Exported function declarations --------------------------------------------*/
extern void Flash_erase(uint32_t address, uint16_t len);
extern int8_t Flash_write_single(uint32_t start_address, uint32_t *buf, uint32_t len);
extern int8_t Flash_write_multi(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
extern void Flash_read(uint32_t address, uint32_t *buf, uint32_t len);
extern uint32_t Get_next_flash_address(uint32_t address);
#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
