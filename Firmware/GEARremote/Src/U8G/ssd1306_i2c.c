/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#include "ssd1306_i2c.h"
#include "main.h"

/* Private variables */
static uint32_t ssd1306_I2C_Timeout;

/* Private defines */
#define I2C_TRANSMITTER_MODE   0
#define I2C_RECEIVER_MODE      1
#define I2C_ACK_ENABLE         1
#define I2C_ACK_DISABLE        0

void ssd1306_I2C_Init() {
	extern I2C_HandleTypeDef hi2c1;
//	GPIO_InitTypeDef gpio;
//	I2C_InitTypeDef i2c;

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

//    i2c.I2C_ClockSpeed = 400000;
//    i2c.I2C_Mode = I2C_Mode_I2C;
//    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
//    i2c.I2C_OwnAddress1 = 0x15;
//    i2c.I2C_Ack = I2C_Ack_Disable;
//    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//    I2C_Init(I2C1, &i2c);

//    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
//    gpio.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB, &gpio);

//    // Ну и включаем, собственно, модуль I2C1
//    I2C_Cmd(I2C1, ENABLE);
	HAL_I2C_Init(&hi2c1);
}





void ssd1306_I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t i;
	ssd1306_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	ssd1306_I2C_WriteData(I2Cx, reg);
	for (i = 0; i < count; i++) {
		ssd1306_I2C_WriteData(I2Cx, data[i]);
	}
	ssd1306_I2C_Stop(I2Cx);
}




/* Private functions */
int16_t ssd1306_I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t ack) {
	/* Generate I2C start pulse */
	I2Cx->CR1 |= I2C_CR1_START;
	
	/* Wait till I2C is busy */
	ssd1306_I2C_Timeout = ssd1306_I2C_TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_SB)) {
		if (--ssd1306_I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Enable ack if we select it */
	if (ack) {
		I2Cx->CR1 |= I2C_CR1_ACK;
	}

	/* Send write/read bit */
	if (direction == I2C_TRANSMITTER_MODE) {
		/* Send address with zero last bit */
		I2Cx->DR = address & ~I2C_OAR1_ADD0;
		
		/* Wait till finished */
		ssd1306_I2C_Timeout = ssd1306_I2C_TIMEOUT;
		while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
			if (--ssd1306_I2C_Timeout == 0x00) {
				return 1;
			}
		}
	}
	if (direction == I2C_RECEIVER_MODE) {
		/* Send address with 1 last bit */
		I2Cx->DR = address | I2C_OAR1_ADD0;
		
		/* Wait till finished */
		ssd1306_I2C_Timeout = ssd1306_I2C_TIMEOUT;
//		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
//			if (--ssd1306_I2C_Timeout == 0x00) {
//				return 1;
//			}
//		}
	}
	
	/* Read status register to clear ADDR flag */
	I2Cx->SR2;
	
	/* Return 0, everything ok */
	return 0;
}

void ssd1306_I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data) {
	/* Wait till I2C is not busy anymore */
	ssd1306_I2C_Timeout = ssd1306_I2C_TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_TXE) && ssd1306_I2C_Timeout) {
		ssd1306_I2C_Timeout--;
	}

	/* Send I2C data */
	I2Cx->DR = data;
}

void ssd1306_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
	ssd1306_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	ssd1306_I2C_WriteData(I2Cx, reg);
	ssd1306_I2C_WriteData(I2Cx, data);
	ssd1306_I2C_Stop(I2Cx);
}


uint8_t ssd1306_I2C_Stop(I2C_TypeDef* I2Cx) {
	/* Wait till transmitter not empty */
	ssd1306_I2C_Timeout = ssd1306_I2C_TIMEOUT;
	while (((!(I2Cx->SR1 & I2C_SR1_TXE)) || (!(I2Cx->SR1 & I2C_SR1_BTF)))) {
		if (--ssd1306_I2C_Timeout == 0x00) {
			return 1;
		}
	}
	
	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;
	
	/* Return 0, everything ok */
	return 0;
}

uint8_t ssd1306_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t address) {
	uint8_t connected = 0;
	/* Try to start, function will return 0 in case device will send ACK */
	if (!ssd1306_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_ENABLE)) {
		connected = 1;
	}

	/* STOP I2C */
	ssd1306_I2C_Stop(I2Cx);

	/* Return status */
	return connected;
}
