/*
 * SHT30_DIS_LIB.h
 *
 *  Created on: Apr 29, 2023
 *      Author: Timurleng44
 */

#ifndef INC_SHT30_DIS_LIB_H_
#define INC_SHT30_DIS_LIB_H_
#include "main.h"
#define SHT30_I2C_ADDR1 (0x44 << 1)
#define SHT30_I2C_ADDR2 (0x45 << 1)
#define SHT30_CHIP_IDENTIFICATION_NUMBER 0x30

#define SHT30_OK  1
#define SHT30_ERR 0
#define SHT30_CRC_OK  1
#define SHT30_CRC_ERR 0


typedef enum CLOCK_STRETCHING_STATUS_ENUM{
	CLOCK_STRETCHING_IS_DISABLE = 0,
	CLOCK_STRETCHING_IS_ENABLE  = 1
}CLOCK_STRETCHING_STATUS;

typedef enum REPEATABILITY_STATUS_ENUM{
	HIGH_REPEATABILITY   = 0,
	MEDIUM_REPEATABILITY = 1,
	LOW_REPEATABILITY    = 2
}REPEATABILITY_STATUS;
typedef enum READ_MODE_ENUM
{
	SINGLE_SHOT_ACQUISITION_MODE = 0,
	SINGLE_SHOT_MODE = 0,
}READ_MODE;
typedef enum MPS_ENUM
{
	MPS_0_5, // 0.5 sec
	MPS_1,   // 1   sec
	MPS_2,   // 2   sec
	MPS_4,   // 4   sec
	MPS_10,  // 10  sec

}MPS;


uint8_t SHT30_Init(void *CommunicationHandleTypeDef);
void SHT30_Soft_Reset();
void SHT30_Enable_Heater();
void SHT30_Disable_Heater();

void SHT30_Single_Shot_Mode_Settings(CLOCK_STRETCHING_STATUS ClockStretchingStatus, REPEATABILITY_STATUS RepeatabilityStatus);
void SHT30_Periodic_Acquisitioun_Mode_Settings(MPS Mps,REPEATABILITY_STATUS RepeatabilityStatus);

uint8_t SHT30_Read_All_With_Single_Shot_Mode();
uint8_t SHT30_Read_All_With_Periodic_Acquisition_Mode();

double SHT30_Get_Temperature_In_Celcius();
double SHT30_Get_Temperature_In_Fahrenheit();
double SHT30_Get_Temperature_In_Kelvin();
double SHT30_Get_Humidity();
uint32_t SHT30_Get_Chip_Identification_Number();
#endif /* INC_SHT30_DIS_LIB_H_ */
