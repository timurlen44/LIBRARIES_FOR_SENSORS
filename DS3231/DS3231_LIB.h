/*
 * DS3231_LIB.h
 *
 *  Created on: Apr 22, 2023
 *      Author: Timurleng44
 */

#ifndef INC_DS3231_LIB_H_
#define INC_DS3231_LIB_H_
#include "main.h"
#define Seconds_Reg 0x00
#define Minutes_Reg 0x01
#define Hours_Reg 0x02
#define Day_Reg 0x03
#define Date_Reg 0x04
#define Month_Reg 0x05
#define Year_Reg 0x06
#define Alarm_1_Seconds_Reg 0x07
#define Alarm_1_Minutes_Reg 0x08
#define Alarm_1_Hours_Reg 0x09
#define Alarm_1_Day_And_Date_Reg 0x0A
#define Alarm_2_Minutes_Reg 0x0B
#define Alarm_2_Hours_Reg 0x0C
#define Alarm_2_Day_And_Date_Reg 0x0D
#define Control_Reg 0x0E
#define Status_Reg 0x0F
#define Aging Offset_Reg 0x10
#define Temperature_MSB_Reg 0x11
#define Temperature_LSB_Reg 0x12
#define DS3231_ADDR1 0x68<<1
#define DS3231_ADDR2 0x50<<1
#define DS3231_ADDR3 0x51<<1
#define DS3231_ADDR4 0x52<<1
#define DS3231_ADDR5 0x53<<1
#define DS3231_ADDR6 0x54<<1
#define DS3231_ADDR7 0x55<<1
#define DS3231_ADDR8 0x56<<1
#define DS3231_ADDR9 0x57<<1

typedef enum days{

	Monday = (uint8_t)1,
	Tuesday = (uint8_t)2,
	Wednesday = (uint8_t)3,
	Thursday = (uint8_t)4,
	Friday = (uint8_t)5,
	Saturday = (uint8_t)6,
	Sunday = (uint8_t)7
}days;

typedef enum months{

	January = (uint8_t) 1,
	February =(uint8_t) 2,
	March = (uint8_t)3,
	April = (uint8_t)4,
	May = (uint8_t)5,
	June = (uint8_t)6,
	July = (uint8_t)7,
	August = (uint8_t)8,
	September = (uint8_t)9,
	October = (uint8_t)10,
	November = (uint8_t)11,
	December = (uint8_t)12
}months;

#define DS3231_OK 1
#define DS3231_ERR 0
uint8_t DS3231_Init(void *CommunicationHandleTypeDef);
void DS3231_Read_All();

void DS3231_Update_Seconds(uint8_t Sec);
uint8_t DS3231_Get_Seconds();

void DS3231_Update_Minutes(uint8_t Min);
uint8_t DS3231_Get_Minutes();

void DS3231_Update_Hours(uint8_t Hours);
uint8_t DS3231_Get_Hours();

void DS3231_Update_Day(uint8_t Day);
uint8_t DS3231_Get_Day();

void DS3231_Update_Date(uint8_t Date);
uint8_t DS3231_Get_Date();

void DS3231_Update_Month(uint8_t Month);
uint8_t DS3231_Get_Month();

void DS3231_Update_Year(uint16_t Year);
uint16_t DS3231_Get_Year();

void DS3231_Update_All(uint8_t Sec, uint8_t Min, uint8_t Hours, uint8_t Day, uint8_t Date, uint8_t Month, uint16_t Year);
void DS3231_Get_All(uint8_t *Sec, uint8_t *Min, uint8_t *Hours, uint8_t *Day, uint8_t *Date, uint8_t *Month, uint16_t *Year);

#endif /* INC_DS3231_LIB_H_ */

