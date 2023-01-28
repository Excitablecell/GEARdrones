/**
 *******************************************************************************
  * Copyright (c) 2022 - ~, SCUT-RobotLab Development Team && EXcai(亢体动画)
  * @file    filter.h
  * @author  SCUT-RobotLab Development Team && EXcai
  * @brief   flash
  *
  ==============================================================================
                            How to use this driver
  ==============================================================================
  @note
	-# 写入数据到片内flash：
	   调用`Flash_write_single()`写入数据到单个扇区。
	   e.g: 
				float writedata[3];	
				Flash_erase(ADDR_FLASH_SECTOR_7, 3);
				Flash_write_single(ADDR_FLASH_SECTOR_7, (uint32_t *)writedata, 3);
	   
	   调用`Flash_write_muli()`写入数据到多个扇区。
	   
	-# 读取片内flash数据： 
	   调用`Flash_read()`读取扇区数据。
	   e.g: 
				float readdata[3];
				Flash_read(ADDR_FLASH_SECTOR_7, (uint32_t *)readdata, 3);
		
  ******************************************************************************
  * @attention
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/  
#include "flash_driver.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
static uint32_t Get_sector(uint32_t address);

/* function prototypes -------------------------------------------------------*/
/**
* @brief  擦除扇区  
* @param  address 扇区地址即扇区号,len为字的长度
* @retval 擦除状态
* @note   建议在通信前单独使用清空板再下载程序，
*/
void Flash_erase(uint32_t address, uint16_t len)
{
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t error;

    flash_erase.Sector = Get_sector(address);
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = len;

    HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
	  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    HAL_FLASHEx_Erase(&flash_erase, &error);
    HAL_FLASH_Lock();
}

/**
 * @brief  写入数据
 * @param  start_address: 写入开始地址，buf：存储数组，len：字的长度
 * @retval 写入状态
 */
int8_t Flash_write_single(uint32_t start_address, uint32_t *buf, uint32_t len)
{
    static uint32_t uw_address;
    static uint32_t end_address;
    static uint32_t *data_buf;
    static uint32_t data_len;

    HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
	  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    uw_address = start_address;
    end_address = Get_next_flash_address(start_address);
    data_buf = buf;
    data_len = 0;

    while (uw_address <= end_address)
    {

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

/**
 * @brief  多个扇区数据，大量数据时使用
 * @param  start_address: 写入开始地址，buf：存储数组，len：字的长度
 * @retval 写入状态
 */
int8_t Flash_write_multi(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len)
{
    uint32_t uw_address = 0;
    uint32_t *data_buf;
    uint32_t data_len;

    HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
	  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    uw_address = start_address;
    data_buf = buf;
    data_len = 0;
    while (uw_address <= end_address)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock(); 
    return 0;
}

/**
  * @brief   从flash当中读取数据
  * @param   start_address: 需读取数据的首位地址 buf：存储数组，len：字的长度
  * @retval  none
  */
void Flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
    memcpy(buf, (void*)address, len *4);
}

/**
  * @brief	获取flash地址的sector号
  * @param	address: flash地址
  * @retval sector号
  */
uint32_t Get_sector(uint32_t address)
{
    uint32_t sector = 0;
    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else 
    {
        sector = FLASH_SECTOR_7;
    }

    return sector;
}

/**
  * @brief	获取下一页flash的地址
  * @param	address:flash的地址
  * @retval 下一页flash的地址
  */
uint32_t Get_next_flash_address(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
    else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_11))*/
    {
        sector = FLASH_END_ADDR;
    }
    return sector;
}
