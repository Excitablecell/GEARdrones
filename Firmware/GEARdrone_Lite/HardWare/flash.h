/*
 * flash.h
 *
 * Created: 2016-04-22
 * Author: zhanglifu
 */
#ifndef _flash_H_
#define _flash_H_


#include "stm32f1xx_hal.h"


/*********************************************************************/
//                        变量定义
/*********************************************************************/
//-- 用途划分
// 0x0800FC00-0x0800FFFF -- 使用最后4k 个字节用来存放电机信息
#define FMC_FLASH_BASE      0x08014000 	// FLASH的起始地址
#define FMC_FLASH_END        0x08015000  // FLASH的结束地址 256



#define FMC_FLASH_SIZE 64			          // 定义Flash大小，单位KB


#if FMC_FLASH_SIZE < 256
#define FMC_SECTOR_SIZE 1024            // 字节
#define MOD_SECTOR_SIZE 0X3FF
#define PAGE_COUNT_BY16 512
#else
#define FMC_SECTOR_SIZE	2048
#define MOD_SECTOR_SIZE 0X7FF
#define PAGE_COUNT_BY16 1024
#endif





/*********************************************************************/
//                        函数声明
/*********************************************************************/
void FlashWriteBuff( const uint32_t destination_address, uint8_t *const buffer,uint32_t length );
void FlashReadBuff(const uint32_t source_address,uint8_t *const buffer,uint16_t length);




#endif
