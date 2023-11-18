/*
 * AT24C256.c
 *
 *  Created on: Nov 15, 2023
 *      Author: Timurleng44
 */

#include "AT24C256.h"


I2C_HandleTypeDef *hi2c;
uint16_t i2cAddress = 0xFFFF;

typedef union floatU8
{
	float f;
	uint8_t u8[4];
}floatU8;

typedef union doubleU8
{
	double d;
	uint8_t u8[8];
}doubleU8;

bool AT24C256_Init(void *CommunicationHandleTypeDef, uint16_t i2cAddress_)
{
	hi2c = (I2C_HandleTypeDef *) CommunicationHandleTypeDef;
	i2cAddress = i2cAddress_;
	return IsDeviceConnected(i2cAddress);
}

bool IsDeviceConnected(uint16_t i2cAddress_)
{
	if(HAL_I2C_IsDeviceReady(hi2c, i2cAddress_, 1, 100) == HAL_OK)
	{
		return true;
	}

	return false;
}

void AT24C256_WriteU8(uint16_t wordAddress, uint8_t data)
{
	uint8_t transmitterBuffer[3] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF), data};
	HAL_I2C_Master_Transmit(hi2c, (i2cAddress & 0xFE), transmitterBuffer, 3, 100);
	while(IsDeviceConnected(i2cAddress) != true);
	return;
}
uint8_t AT24C256_ReadU8(uint16_t wordAddress)
{
	uint8_t data = 0x00;
	uint8_t transmitterBuffer[2] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF)};
	HAL_I2C_Master_Transmit(hi2c, i2cAddress, transmitterBuffer, 2, 100);
	HAL_I2C_Master_Receive(hi2c, (i2cAddress | 0x01), &data, 1, 100);
	return data;
}

void AT24C256_WriteU16(uint16_t wordAddress, uint16_t data)
{
	uint8_t transmitterBuffer[4] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF), ((data>>8) & 0xFF), (data & 0xFF) };
	HAL_I2C_Master_Transmit(hi2c, (i2cAddress & 0xFE), transmitterBuffer, 4, 100);
	while(IsDeviceConnected(i2cAddress) != true);
	return;
}
uint16_t AT24C256_ReadU16(uint16_t wordAddress)
{
	uint16_t data = 0x00;
	uint8_t receiverBuffer[2];
	uint8_t transmitterBuffer[2] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF)};
	HAL_I2C_Master_Transmit(hi2c, i2cAddress, transmitterBuffer, 2, 100);
	HAL_I2C_Master_Receive(hi2c, (i2cAddress | 0x01), receiverBuffer, 2, 100);
	data = ((uint16_t)receiverBuffer[0]<<8) | ((uint16_t)receiverBuffer[1]);
	return data;
}

void AT24C256_WriteU32(uint16_t wordAddress, uint32_t data)
{
	uint8_t transmitterBuffer[6] = {((wordAddress>>8) & 0xFF),(wordAddress & 0xFF), ((data>>24) & 0xFF), ((data>>16) & 0xFF), ((data>>8) & 0xFF), (data & 0xFF) };
	HAL_I2C_Master_Transmit(hi2c, (i2cAddress & 0xFE), transmitterBuffer, 6, 100);
	while(IsDeviceConnected(i2cAddress) != true);
	return;
}
uint32_t AT24C256_ReadU32(uint16_t wordAddress)
{
	uint32_t data = 0x00;
	uint8_t receiverBuffer[4];
	uint8_t transmitterBuffer[2] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF)};
	HAL_I2C_Master_Transmit(hi2c, i2cAddress, transmitterBuffer, 2, 100);
	HAL_I2C_Master_Receive(hi2c, (i2cAddress | 0x01), receiverBuffer, 4, 100);
	data = ((uint32_t)receiverBuffer[0]<<24) | ((uint32_t)receiverBuffer[1]<<16) | ((uint32_t)receiverBuffer[2]<<8) | ((uint32_t)receiverBuffer[3]);
	return data;
}

void AT24C256_WriteFloat(uint16_t wordAddress, float data)
{
	floatU8 tmp;
	tmp.f = data;
	uint8_t transmitterBuffer[6] = {((wordAddress>>8) & 0xFF),(wordAddress & 0xFF), tmp.u8[0], tmp.u8[1], tmp.u8[2], tmp.u8[3]};
	HAL_I2C_Master_Transmit(hi2c, (i2cAddress & 0xFE), transmitterBuffer, 6, 100);
	while(IsDeviceConnected(i2cAddress) != true);
	return;
}
float AT24C256_ReadFloat(uint16_t wordAddress)
{
	floatU8 tmp;
	uint8_t receiverBuffer[4];
	uint8_t transmitterBuffer[2] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF)};
	HAL_I2C_Master_Transmit(hi2c, i2cAddress, transmitterBuffer, 2, 100);
	HAL_I2C_Master_Receive(hi2c, (i2cAddress | 0x01), receiverBuffer, 4, 100);
	tmp.u8[0] = receiverBuffer[0];
	tmp.u8[1] = receiverBuffer[1];
	tmp.u8[2] = receiverBuffer[2];
	tmp.u8[3] = receiverBuffer[3];
	return tmp.f;
}
void AT24C256_WriteDouble(uint16_t wordAddress, double data)
{
	doubleU8 tmp;
	tmp.d = data;
	uint8_t transmitterBuffer[10] = {((wordAddress>>8) & 0xFF),(wordAddress & 0xFF), tmp.u8[0], tmp.u8[1], tmp.u8[2], tmp.u8[3], tmp.u8[4], tmp.u8[5], tmp.u8[6], tmp.u8[7]};
	HAL_I2C_Master_Transmit(hi2c, (i2cAddress & 0xFE), transmitterBuffer, 10, 100);
	while(IsDeviceConnected(i2cAddress) != true);
	return;
}
double AT24C256_ReadDouble(uint16_t wordAddress)
{
	doubleU8 tmp;
	uint8_t receiverBuffer[8];
	uint8_t transmitterBuffer[2] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF)};
	HAL_I2C_Master_Transmit(hi2c, i2cAddress, transmitterBuffer, 2, 100);
	HAL_I2C_Master_Receive(hi2c, (i2cAddress | 0x01), receiverBuffer, 8, 100);
	tmp.u8[0] = receiverBuffer[0];
	tmp.u8[1] = receiverBuffer[1];
	tmp.u8[2] = receiverBuffer[2];
	tmp.u8[3] = receiverBuffer[3];
	tmp.u8[4] = receiverBuffer[4];
	tmp.u8[5] = receiverBuffer[5];
	tmp.u8[6] = receiverBuffer[6];
	tmp.u8[7] = receiverBuffer[7];
	return tmp.d;
}

void AT24C256_WriteChar(uint16_t wordAddress, char data)
{
	uint8_t transmitterBuffer[3] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF), (uint8_t)data};
	HAL_I2C_Master_Transmit(hi2c, (i2cAddress & 0xFE), transmitterBuffer, 3, 100);
	while(IsDeviceConnected(i2cAddress) != true);
	return;
}
char AT24C256_ReadChar(uint16_t wordAddress)
{
	char data = 0x00;
	uint8_t transmitterBuffer[2] = {((wordAddress>>8) & 0xFF), (wordAddress & 0xFF)};
	HAL_I2C_Master_Transmit(hi2c, i2cAddress, transmitterBuffer, 2, 100);
	HAL_I2C_Master_Receive(hi2c, (i2cAddress | 0x01), (uint8_t *)&data, 1, 100);
	return data;
}
