/*
 * DS3231_LIB.c
 *
 *  Created on: Apr 22, 2023
 *      Author: Timurleng44
 */
#include "DS3231_LIB.h"
uint8_t DS3231_Read_Register_1_Byte(uint8_t Reg_Addr);
void DS3231_Read_Register_n_Byte(uint8_t Reg_Addr, uint8_t n, uint8_t *buff);
void DS3231_Write_Register_1_Byte(uint8_t Reg_Addr, uint8_t  Reg_Val);

static uint8_t DS3231_Find_I2C_Addr();
static uint8_t bcd_to_dec(uint8_t bcd);
static uint8_t dec_to_bcd(uint8_t dec);

typedef struct Temperature
{
	int8_t temp_msb;
	int8_t temp_lsb;
	uint16_t raw_temp;
	double C;
	double F;
	double K;
}Temp;



typedef struct Time
{
	uint8_t Senconds;
	uint8_t Minutes;
	uint8_t Hours;
	days Day;
	uint8_t Date;
	months Month;
	uint16_t Year;
}Time;

struct DS3231
{
	uint8_t DS3231_I2C_ADDR;
	Temp Temp;
	Time Time;
}DS3231;


I2C_HandleTypeDef *hi2c;


uint8_t DS3231_Init(void *CommunicationHandleTypeDef)
{

	hi2c = (I2C_HandleTypeDef *)CommunicationHandleTypeDef;
	if(DS3231_Find_I2C_Addr() == DS3231_OK)
	{
		uint8_t hours_reg_val = 0;
		hours_reg_val = DS3231_Read_Register_1_Byte(Hours_Reg) & 0x3F;
		DS3231_Write_Register_1_Byte(Hours_Reg, hours_reg_val);
		return DS3231_OK;
	}

	return DS3231_ERR;
}

void DS3231_Read_All()
{
	uint8_t buff[9] ={0};
	DS3231_Read_Register_n_Byte(Seconds_Reg, 7, buff);
	DS3231_Read_Register_n_Byte(Temperature_MSB_Reg, 2, &buff[7]);

	DS3231.Time.Senconds = bcd_to_dec(buff[0]);
	DS3231.Time.Minutes = bcd_to_dec(buff[1]);
	DS3231.Time.Hours = bcd_to_dec(buff[2]);
	DS3231.Time.Day = bcd_to_dec(buff[3]);
	DS3231.Time.Date = bcd_to_dec(buff[4]);
	DS3231.Time.Month = bcd_to_dec(buff[5]);
	DS3231.Time.Year = bcd_to_dec(buff[6]) + 2000;

	DS3231.Temp.temp_msb = buff[7];
	DS3231.Temp.temp_lsb = buff[8];
	DS3231.Temp.raw_temp = (DS3231.Temp.temp_msb<<8)| (DS3231.Temp.temp_lsb & 0xC0);
	DS3231.Temp.C = DS3231.Temp.raw_temp/256.0;
	DS3231.Temp.K = DS3231.Temp.C + 273.15;
	DS3231.Temp.F = ((DS3231.Temp.C*1.8)+32.0);
	return;
}

void DS3231_Update_Seconds(uint8_t Sec)
{
	uint8_t encoded_sec_in_bcd = dec_to_bcd(Sec);
	DS3231_Write_Register_1_Byte(Seconds_Reg, encoded_sec_in_bcd);
	return;
}
uint8_t DS3231_Get_Seconds()
{
	return DS3231.Time.Senconds;
}
void DS3231_Update_Minutes(uint8_t Min)
{
	uint8_t encoded_min_in_bcd = dec_to_bcd(Min);
	DS3231_Write_Register_1_Byte(Minutes_Reg, encoded_min_in_bcd);
	return;
}
uint8_t DS3231_Get_Minutes()
{
	return DS3231.Time.Minutes;
}

void DS3231_Update_Hours(uint8_t Hours)
{
	uint8_t encoded_hours_in_bcd = dec_to_bcd(Hours);
	DS3231_Write_Register_1_Byte(Hours_Reg, encoded_hours_in_bcd);
	return;
}
uint8_t DS3231_Get_Hours()
{
	return DS3231.Time.Hours;
}


void DS3231_Update_Day(uint8_t Day)
{
	uint8_t encoded_day_in_bcd = dec_to_bcd(Day);
	DS3231_Write_Register_1_Byte(Day_Reg, encoded_day_in_bcd);
	return;
}
uint8_t DS3231_Get_Day()
{
	return DS3231.Time.Day;
}
void DS3231_Update_Date(uint8_t Date)
{
	uint8_t encoded_date_in_bcd = dec_to_bcd(Date);
	DS3231_Write_Register_1_Byte(Date_Reg, encoded_date_in_bcd);
	return;
}
uint8_t DS3231_Get_Date()
{
	return DS3231.Time.Date;
}

void DS3231_Update_Month(uint8_t Month)
{
	uint8_t encoded_month_in_bcd = dec_to_bcd(Month);
	DS3231_Write_Register_1_Byte(Month_Reg, encoded_month_in_bcd);
	return;
}
uint8_t DS3231_Get_Month()
{
	return DS3231.Time.Month;
}

void DS3231_Update_Year(uint16_t Year)
{
	uint8_t year_ui8 =(uint8_t) (Year - 2000);
	uint8_t encoded_year_in_bcd = dec_to_bcd(year_ui8);
	DS3231_Write_Register_1_Byte(Year_Reg, encoded_year_in_bcd);
	return;
}
uint16_t DS3231_Get_Year()
{
	return DS3231.Time.Year;
}

void DS3231_Update_All(uint8_t Sec, uint8_t Min, uint8_t Hours, uint8_t Day, uint8_t Date, uint8_t Month, uint16_t Year)
{
	DS3231_Update_Seconds(Sec);
	DS3231_Update_Minutes(Min);
	DS3231_Update_Hours(Hours);
	DS3231_Update_Day(Day);
	DS3231_Update_Date(Date);
	DS3231_Update_Month(Month);
	DS3231_Update_Year(Year);
	return;
}

void DS3231_Get_All(uint8_t *Sec, uint8_t *Min, uint8_t *Hours, uint8_t *Day, uint8_t *Date, uint8_t *Month, uint16_t *Year)
{
	*Sec   = DS3231_Get_Seconds();
	*Min   = DS3231_Get_Minutes();
	*Hours = DS3231_Get_Hours();
	*Day   = DS3231_Get_Day();
	*Date  = DS3231_Get_Date();
	*Month = DS3231_Get_Month();
	*Year  = DS3231_Get_Year();
	return;
}



uint8_t DS3231_Read_Register_1_Byte(uint8_t Reg_Addr)
{
	uint8_t Reg_Val = 0;
	HAL_I2C_Mem_Read(hi2c, DS3231.DS3231_I2C_ADDR, Reg_Addr, 1, &Reg_Val, 1, 100);
	return Reg_Val;
}

void DS3231_Read_Register_n_Byte(uint8_t Reg_Addr, uint8_t n, uint8_t *buff)
{
	HAL_I2C_Mem_Read(hi2c,DS3231.DS3231_I2C_ADDR , Reg_Addr, 1, &buff[0], n, 100);
	return;
}

void DS3231_Write_Register_1_Byte(uint8_t Reg_Addr, uint8_t  Reg_Val)
{
	HAL_I2C_Mem_Write(hi2c, DS3231.DS3231_I2C_ADDR ,Reg_Addr,1,&Reg_Val,1,100);
	return;
}

static uint8_t DS3231_Find_I2C_Addr()
{
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR1,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR1;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR2,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR2;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR3,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR3;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR4,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR4;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR5,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR5;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR6,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR6;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR7,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR7;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR8,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR8;
		return DS3231_OK;
	}
	if(HAL_I2C_IsDeviceReady(hi2c,DS3231_ADDR9,1,100) == HAL_OK)
	{
		DS3231.DS3231_I2C_ADDR = DS3231_ADDR9;
		return DS3231_OK;
	}
	return DS3231_ERR;

}


static uint8_t bcd_to_dec(uint8_t bcd)
{
	uint8_t dec;
	dec = (bcd & 0x0F) + ((bcd>>4) * 10);
	return dec;
}
static uint8_t dec_to_bcd(uint8_t dec)
{
	uint8_t bcd;
	bcd = (dec%10) | ((dec/10)<<4);
	return bcd;
}
