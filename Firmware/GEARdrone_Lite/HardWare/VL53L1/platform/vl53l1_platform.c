
/*
* @file	   vl53l1_platform.c
* @brief    针对HAL库与模拟IIC的平台兼容文件
* @details  This is the detail description. 
* @author   WMD
* @date     date 
* @version  0.1
* @par Copyright (c):  
*       WMD 
* @par 日志 2020年4月16日13:40:46
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/


#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "i2c.h"
#include <string.h>

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    if(IICwriteBytes(Dev->I2cDevAddr,index,count,pdata)==count)
    return VL53L1_ERROR_NONE;
    else return VL53L1_ERROR_CONTROL_INTERFACE;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    if(IICreadBytes(Dev->I2cDevAddr,index,count,pdata)==count)
    return VL53L1_ERROR_NONE;
    else return VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    if(IICwriteByte(Dev->I2cDevAddr,index,data))
    return VL53L1_ERROR_NONE;
    else return VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    IICwriteByte(Dev->I2cDevAddr,index,data>>8);
    IICwriteByte(Dev->I2cDevAddr,index,data & 0x00ff);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
    IICwriteByte(Dev->I2cDevAddr,index,data>>24);
    IICwriteByte(Dev->I2cDevAddr,index,(data >> 16) & 0xff);
    IICwriteByte(Dev->I2cDevAddr,index,(data >> 8) & 0xff);
    IICwriteByte(Dev->I2cDevAddr,index,(data >> 0) & 0xff);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    uint8_t data;
    VL53L1_Error states=VL53L1_ERROR_NONE;
    if(!IIC_ReadOneByte(Dev->I2cDevAddr,index,&data))
    {
        states=VL53L1_ERROR_CONTROL_INTERFACE;
    }
    data=(data & AndData) | OrData;
    if(!IICwriteByte(Dev->I2cDevAddr,index,data))
    {
        states=VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return states;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
    if(IIC_ReadOneByte(Dev->I2cDevAddr,index,data))
    {
        return VL53L1_ERROR_NONE;
    }else return VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
    uint8_t ret[2];
    IICreadBytes(Dev->I2cDevAddr,index,2,(uint8_t*)ret);
    *data=(ret[0]<<8) | ret[1];
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
    uint8_t ret[4];
    IICreadBytes(Dev->I2cDevAddr,index,4,(uint8_t*)ret);
    *data=(ret[0]<<24) | (ret[1]<<16) | (ret[2]<<8) | ret[3];
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	*ptick_count_ms=HAL_GetTick();
	return VL53L1_ERROR_NONE;;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz=10000000;
	return VL53L1_ERROR_NONE;;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	HAL_Delay(wait_ms);
	return VL53L1_ERROR_NONE;;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
    //这里使用不精确的循环延时代替，省去一个定时器
    wait_us=wait_us*72;
    while(wait_us)wait_us--;
	return VL53L1_ERROR_NONE;;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;;
	uint32_t     polling_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;

	/* wait until value is found, timeout reached on error occurred */
	while ((status == VL53L1_ERROR_NONE)
					&& (polling_time_ms < timeout_ms)
					&& (found == 0)) 
	{
		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_RdByte(pdev, index,	&byte_value);

		if ((byte_value & mask) == value)
			found = 1;

		if (status == VL53L1_ERROR_NONE	&& found == 0 && poll_delay_ms > 0)
			status = VL53L1_WaitMs(pdev, poll_delay_ms);

		/* Update polling time */
		polling_time_ms++;
	}

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}




