/*
 * AT24C256.h
 *
 *  Created on: Nov 15, 2023
 *      Author: Timurleng44
 */

#ifndef INC_AT24C256_H_
#define INC_AT24C256_H_
#include <stdbool.h>
//#include <stdint.h>
#include "main.h"

bool AT24C256_Init(void *CommunicationHandleTypeDef, uint16_t i2cAddress);
bool IsDeviceConnected(uint16_t i2cAddress);

void AT24C256_WriteU8(uint16_t wordAddress, uint8_t data);
uint8_t AT24C256_ReadU8(uint16_t wordAddress);

void AT24C256_WriteU16(uint16_t wordAddress, uint16_t data);
uint16_t AT24C256_ReadU16(uint16_t wordAddress);

void AT24C256_WriteU32(uint16_t wordAddress, uint32_t data);
uint32_t AT24C256_ReadU32(uint16_t wordAddress);

void AT24C256_WriteFloat(uint16_t wordAddress, float data);
float AT24C256_ReadFloat(uint16_t wordAddress);

void AT24C256_WriteDouble(uint16_t wordAddress, double data);
double AT24C256_ReadDouble(uint16_t wordAddress);

void AT24C256_WriteChar(uint16_t wordAddress, char data);
char AT24C256_ReadChar(uint16_t wordAddress);

#endif /* INC_AT24C256_H_ */
