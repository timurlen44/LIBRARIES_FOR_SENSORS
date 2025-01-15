/*
 * AS5600.c
 *
 *  Created on: Jan 14, 2025
 *      Author: Emin
 */

#include "AS5600.h"

static I2C_HandleTypeDef *hi2c = NULL;

static void AS5600_write1Byte(uint8_t regAddr_u8, uint8_t data_u8);
static void AS5600_write2Bytes(uint8_t regAddr_u8, uint16_t data_u16);

static uint8_t  AS5600_read1Byte(uint8_t regAddr_u8);
static uint16_t AS5600_read2Bytes(uint8_t regAddr_u8);

bool AS5600_init(I2C_HandleTypeDef  *communicationHandleTypeDef)
{
	hi2c = communicationHandleTypeDef;
	return AS5600_isDeviceConnectedToTheI2CLine();
}

bool AS5600_isDeviceConnectedToTheI2CLine()
{
	if(HAL_I2C_IsDeviceReady(hi2c,AS5600_ADDRESS,1,100) == HAL_OK)
		return AS5600_IS_CONNECTED;
	else
		return AS5600_IS_NOT_CONNECTED;
}

MagnetDetectionStatus_e AS5600_readMagnetDetectionStatus()
{
	StatusRegisterData_u statusRegister;
	statusRegister.statusRegisterVal_u8 = AS5600_read1Byte(STATUS_REG);

	if(statusRegister.bits.MD)
	{
		return MAGNET_IS_DETECTED;
	}
	else
	{
		return MAGNET_IS_NOT_DETECTED;
	}

}

MagnetStrengthStatus_e AS5600_readMagnetStrength()
{
	StatusRegisterData_u statusRegister;
	statusRegister.statusRegisterVal_u8 = AS5600_read1Byte(STATUS_REG);

	if(statusRegister.bits.MH)
	{
		return MAGNET_IS_TOO_STRONG;
	}
	if(statusRegister.bits.ML)
	{
		return MAGNET_IS_TOO_WEAK;
	}

	return MAGNET_IS_NORMAL;
}

void     AS5600_setZeroPosition(uint16_t currentRawAngle_u16)
{
	AS5600_write2Bytes(ZPOS_REG_ADDR, currentRawAngle_u16);
	HAL_Delay(1);
}

uint16_t AS5600_readRawAngle()
{
	uint16_t rawAngle_u16 = AS5600_read2Bytes(RAW_ANGLE_REG_ADDR);
	return rawAngle_u16;
}
uint16_t AS5600_readAngle()
{
	uint16_t rawAngle_u16 = AS5600_read2Bytes(ANGLE_REG_ADDR);
	return rawAngle_u16;
}
float    AS5600_readFloatAngle()
{
	float    angle_f32 = 0;
	uint16_t angle_u16 = AS5600_readAngle();
	angle_f32 = (angle_u16 * 360) /4095.0;
	return angle_f32;
}

void AS5600_write1Byte(uint8_t regAddr_u8, uint8_t data_u8)
{
	HAL_I2C_Mem_Write(hi2c, AS5600_ADDRESS, regAddr_u8, 1, &data_u8, 1, 1000);
}
void AS5600_write2Bytes(uint8_t regAddr_u8, uint16_t data_u16)
{
	uint8_t byte1 = (data_u16 >> 8);
	uint8_t byte2 = (data_u16 & 0xFF);
	uint8_t data[2] = {byte1, byte2};
	HAL_I2C_Mem_Write(hi2c, AS5600_ADDRESS, regAddr_u8, 1, data, 2, 1000);
}

uint8_t  AS5600_read1Byte(uint8_t regAddr_u8)
{
	uint8_t data_u8 = 0;
	HAL_I2C_Mem_Read(hi2c, AS5600_ADDRESS, regAddr_u8, 1, &data_u8, 1, 1000);
	return data_u8;
}
uint16_t AS5600_read2Bytes(uint8_t regAddr_u8)
{
	uint8_t data[2];
	uint16_t data_u16 = 0;
	HAL_I2C_Mem_Read(hi2c, AS5600_ADDRESS, regAddr_u8, 1,data, 2, 1000);
	data_u16 = (((uint16_t)data[0]) << 8) | (data[1]);
	return data_u16;
}
