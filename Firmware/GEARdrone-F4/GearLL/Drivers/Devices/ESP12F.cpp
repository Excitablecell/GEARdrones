/********************************************************************************
  * Copyright (c) 2022 - ~, EXcai(亢体造梦)
  * @file    ESP12F.h
  * @author  EXcai
  * @brief   ESP8266 MQTT固件驱动
  *
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *

  *******************************************************************************/
	/* Includes ------------------------------------------------------------------*/
#include "ESP12F.h"

uint8_t STATE[60] = "AT\r\n";
uint8_t RST[60] = "AT+RST\r\n";
uint8_t WIFI_MODE_SET[60] = "AT+CWMODE=1\r\n";
uint8_t WIFI_CONFIG[40] = "AT+CWJAP=\"GEARMAN-LAPTOP\",\"66999879\"\n";
uint8_t MQTT_CONFIG[60] = "AT+MQTTUSERCFG=0,1,'MAV1','admin','public',0,0,''\r\n";
uint8_t MQTT_CONNECT[60] = "AT+MQTTCONN=0,'192.168.137.1',1883,0\r\n";

uint8_t MQTT_SEND[60] = "AT+MQTTPUB=0,'MAV1_TX','TESTtest123',2,0\r\n";
uint8_t MQTT_SUB[60] = "AT+MQTTSUB=0,'MAV1_RX',2\r\n";
uint8_t MQTT_REMOTE[60] = "AT+MQTTSUB=0,'REMOTE',2\r\n";
/* function prototypes -------------------------------------------------------*/
