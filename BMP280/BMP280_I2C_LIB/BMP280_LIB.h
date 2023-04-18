/*
 * BMP280_LIB.h
 *
 *  Created on: Apr 10, 2023
 *      Author: Timurleng44
 */

#ifndef INC_BMP280_LIB_H_
#define INC_BMP280_LIB_H_
#include "stdint.h"
#define BMP280_I2C_ADDR1 0xEE //(0x77)<<1
#define BMP280_I2C_ADDR2 0xEC //(0x76)<<1
#define BMP280_CHIP_IDENTIFICATION_NUMBER 0x58

#define BMP280_OK  1
#define BMP280_ERR 0

//Address of registers
#define temp_xlsb  0xFC
#define temp_lsb   0xFB
#define temp_msb   0xFA
#define press_xlsb 0xF9
#define press_lsb  0xF8
#define press_msb  0xF7
#define config     0xF5
#define ctrl_meas  0xF4
#define status     0xF3
#define reset      0xE0
#define id         0xD0
#define calib25    0xA1
#define calib00    0x88
//Address of compensation parameter storage
#define dig_T1_Addr 0x88 // uint16_t
#define dig_T2_Addr 0x8A // int16_t
#define dig_T3_Addr 0x8C // int16_t
#define dig_P1_Addr 0x8E // uint16_t
#define dig_P2_Addr 0x90 // int16_t
#define dig_P3_Addr 0x92 // int16_t
#define dig_P4_Addr 0x94 // int16_t
#define dig_P5_Addr 0x96 // int16_t
#define dig_P6_Addr 0x98 // int16_t
#define dig_P7_Addr 0x9A // int16_t
#define dig_P8_Addr 0x9C // int16_t
#define dig_P9_Addr 0x9E // int16_t


//Commands
#define SOFT_RESET_COMMAND 0xB6
typedef enum BMP280_MODE{
	Sleep_Mode,
	Forced_Mode,
	Normal_Mode,
}BMP280_MODE;

typedef enum BMP280_OSRSP{
	osrsp_x16,
	osrsp_x8,
	osrsp_x4,
	osrsp_x2,
	osrsp_x1
}BMP280_OSRSP;

typedef enum BMP280_OSRST{
	osrst_x16,
	osrst_x8,
	osrst_x4,
	osrst_x2,
	osrst_x1
}BMP280_OSRST;

typedef enum BMP280_IIR_FILTER_COEFFICIENT{
	IIR_F_C_16,
	IIR_F_C_8,
	IIR_F_C_4,
	IIR_F_C_2,
	IIR_F_C_Filter_Off,
}BMP280_IIR_FILTER_COEFFICIENT;

typedef enum BMP280_STANDBY_TIME{
	t_sb_0_5 ,
	t_sb_62_5,
	t_sb_125 ,
	t_sb_250 ,
	t_sb_500 ,
	t_sb_1000,
	t_sb_2000,
	t_sb_4000
}BMP280_STANDBY_TIME;

typedef enum USE_CASE{              		//Mode        over-sampling          osrs_p        osrs_t     IIR Filter Coefficient     Timing          ODR (Hz)
	Handheld_device_low_power_case, 		//Normal      ultra high resolution  x16           x2         4 						 62.5 ms		 10
	Handheld_device_dynamic_case,           //Normal      standard resolution    x4            x1         16 						 0.50 ms		 83.3
	Weather_monitoring_case,                //Forced      ultra low power        x1            x1         Off 						 1.00 min		 1/60
	Elevator_floor_change_detection_case,   //Normal      standard resolution    x4            x1         4 						 125  ms		 7.30
	Drop_detection_case,                    //Normal      low power              x2            x1         Off 						 0.5  ms		 125
	Indoor_navigation_case,                 //Normal      ultra high resolution  x16           x2         16 						 0.5  ms		 26.0
	Custom_case
}USE_CASE;

uint8_t BMP280_Init(void *CommunicationHandleTypeDef,USE_CASE Use_Case);//Initialize BMP280
uint8_t BMP280_Check_Is_Device_Connected();//if BMP280 Connected to microcontroller then return BMP280_OK else BMP280_ERR
uint8_t BMP280_Get_Chip_Identification_Number(); // Return the Chip ID
void BMP280_Set_Using_Settings(BMP280_MODE M,BMP280_OSRSP OSRS_P,BMP280_OSRST OSRS_T, BMP280_IIR_FILTER_COEFFICIENT IIR_F_C, BMP280_STANDBY_TIME T_SB);
void BMP280_Read_All(void);//Read pressure and temperature

double BMP280_Get_Temperature_In_Celcius();
double BMP280_Get_Temperature_In_Kelvin();
double BMP280_Get_Temperature_In_Fahrenheit();
double BMP280_Get_Altitude();
double BMP280_Set_Pressure_In_Sea_Level(double hPa_in_sea_level);
double BMP280_Get_Pressure_In_Pa();
double BMP280_Get_Temperature_In_hPa();
//uint32_t BMP280_Read_Register_3_Bytes(uint8_t Reg_Addr);
#endif /* INC_BMP280_LIB_H_ */
