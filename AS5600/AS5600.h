/*
 * AS5600.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Emin
 */

#ifndef AS5600_AS5600_H_
#define AS5600_AS5600_H_

#include "main.h"
#include "stdbool.h"
//AS5600 I2C SLAVE ADDRESS
#define AS5600_ADDRESS (0x36 << 1)

/*   AS5600 REGISTERS   */

//CONFIGURATION REGISTERS
#define ZPOS_REG_ADDR 0x01

//OUTPUT REGISTERS
#define RAW_ANGLE_REG_ADDR 0x0C
#define ANGLE_REG_ADDR 0x0E

// STATUS REGISTERS
#define STATUS_REG 0x0B

typedef enum MagnetDetectionStatus
{
	MAGNET_IS_DETECTED     = 0x04,
	MAGNET_IS_NOT_DETECTED = 0x05
}MagnetDetectionStatus_e;

typedef enum MagnetStrengthStatus
{
	MAGNET_IS_TOO_STRONG   = 0x01,
	MAGNET_IS_TOO_WEAK     = 0x02,
	MAGNET_IS_NORMAL       = 0x03
}MagnetStrengthStatus_e;


typedef enum AS5600ConnectionStatus
{
	AS5600_IS_NOT_CONNECTED = 0x00,
	AS5600_IS_CONNECTED 	= 0x01
}AS5600ConnectionStatus_e;

typedef union StatusRegisterData{
	struct{
		uint8_t r1:1;
		uint8_t r2:1;
		uint8_t MD:1;
		uint8_t ML:1;
		uint8_t MH:1;
		uint8_t r6:1;
		uint8_t r7:1;
		uint8_t r8:1;
	}bits;
	uint8_t statusRegisterVal_u8;
}StatusRegisterData_u;


bool AS5600_init(I2C_HandleTypeDef  *communicationHandleTypeDef);
bool AS5600_isDeviceConnectedToTheI2CLine();
MagnetDetectionStatus_e AS5600_readMagnetDetectionStatus();
MagnetStrengthStatus_e  AS5600_readMagnetStrength();

void     AS5600_setZeroPosition(uint16_t currentRawAngle_u16);

uint16_t AS5600_readRawAngle();
uint16_t AS5600_readAngle();
float    AS5600_readFloatAngle();

#endif /* AS5600_AS5600_H_ */
