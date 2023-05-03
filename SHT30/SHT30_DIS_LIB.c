/*
 * SHT30_DIS_LIB.c
 *
 *  Created on: Apr 29, 2023
 *      Author: Timurleng44
 */
#include "SHT30_DIS_LIB.h"

#include "main.h"
static I2C_HandleTypeDef *hi2c;
void SHT30_Write_Command_2_Byte(uint8_t *data);
void SHT30_Read_N_Byte(uint8_t *data, uint8_t size);
uint8_t SHT30_i2c_write_operation(uint8_t addr);
uint8_t SHT30_i2c_read_operation(uint8_t addr);
uint32_t SHT30_Read_Chip_Identification_Number();
uint8_t SHT30_Check_Is_Device_Connected();
uint8_t SHT30_CRC8_Calculator(const uint8_t *idata, int len);

struct SHT30_DIS{
	uint16_t I2C_ADDR;
	uint32_t CHIP_ID;
	double Celcius;
	double Fahrenheit;
	double Kelvin;
	double Humidity;

	uint8_t single_shot_mode_command[2];
}SHT30;

void SHT30_Single_Shot_Mode_Settings(CLOCK_STRETCHING_STATUS ClockStretchingStatus, REPEATABILITY_STATUS RepeatabilityStatus)
{
	uint8_t Command[2] = {0x00,0x00};
	if(ClockStretchingStatus == CLOCK_STRETCHING_IS_ENABLE)
	{
		Command[0] = 0x2C;
		if(RepeatabilityStatus == HIGH_REPEATABILITY)
		{
			Command[1] = 0x06;
		}
		else if(RepeatabilityStatus == MEDIUM_REPEATABILITY)
		{
			Command[1] = 0x0D;
		}
		else if(RepeatabilityStatus == LOW_REPEATABILITY)
		{
			Command[1] = 0x10;
		}
		else
		{
			return;
		}

	}
	else if(ClockStretchingStatus == CLOCK_STRETCHING_IS_DISABLE) // Clock stretching is disable
	{
		Command[0] = 0x24;
		if(RepeatabilityStatus == HIGH_REPEATABILITY)
		{
			Command[1] = 0x00;
		}
		else if(RepeatabilityStatus == MEDIUM_REPEATABILITY)
		{
			Command[1] = 0x0B;
		}
		else if(RepeatabilityStatus == LOW_REPEATABILITY)
		{
			Command[1] = 0x16;
		}
		else
		{
			return;
		}
	}
	else
	{
		return;
	}
	SHT30.single_shot_mode_command[0] = Command[0];
	SHT30.single_shot_mode_command[1] = Command[1];
	SHT30_Write_Command_2_Byte(Command);
	return;
}
uint8_t SHT30_Read_All_With_Single_Shot_Mode()
{
	SHT30_Write_Command_2_Byte(SHT30.single_shot_mode_command);
	uint8_t RawTempAndHum[6]={0};
	SHT30_Read_N_Byte(RawTempAndHum, 6);

	uint8_t crc1 = SHT30_CRC8_Calculator(&RawTempAndHum[0], 2);
	if(crc1 != RawTempAndHum[2])
	{
		return SHT30_ERR;
	}

	uint8_t crc2 = SHT30_CRC8_Calculator(&RawTempAndHum[3], 2);
	if(crc2 != RawTempAndHum[5])
	{
		return SHT30_ERR;
	}

	uint16_t raw_temp = (RawTempAndHum[0]<<8)|(RawTempAndHum[1]<<0);
	uint16_t raw_hum = (RawTempAndHum[3]<<8)|(RawTempAndHum[4]<<0);

	SHT30.Celcius = -45.0 + 175.0*raw_temp/65535.0;
	SHT30.Fahrenheit = -49.0 + 315.0*raw_temp/65535.0;
	SHT30.Kelvin = SHT30.Celcius + 273.15;
	SHT30.Humidity = 100.0*raw_hum/65535.0;

	return SHT30_OK;
}

void SHT30_Periodic_Acquisitioun_Mode_Settings(MPS Mps,REPEATABILITY_STATUS RepeatabilityStatus)
{
	uint8_t Command[2]={0};
	if(Mps == MPS_0_5)
	{
		Command[0] = 0x20;
		if(RepeatabilityStatus == HIGH_REPEATABILITY)
		{
			Command[1] = 0x32;
		}
		else if(RepeatabilityStatus == MEDIUM_REPEATABILITY)
		{
			Command[1] = 0x24;
		}
		else if(RepeatabilityStatus == LOW_REPEATABILITY)
		{
			Command[1] = 0x2F;
		}
		else
		{
			return;
		}
	}
	else if(Mps == MPS_1)
	{
		Command[0] = 0x21;
		if(RepeatabilityStatus == HIGH_REPEATABILITY)
		{
			Command[1] = 0x30;
		}
		else if(RepeatabilityStatus == MEDIUM_REPEATABILITY)
		{
			Command[1] = 0x26;
		}
		else if(RepeatabilityStatus == LOW_REPEATABILITY)
		{
			Command[1] = 0x2D;
		}
		else
		{
			return;
		}

	}
	else if(Mps == MPS_2)
	{
		Command[0] = 0x22;
		if(RepeatabilityStatus == HIGH_REPEATABILITY)
		{
			Command[1] = 0x36;
		}
		else if(RepeatabilityStatus == MEDIUM_REPEATABILITY)
		{
			Command[1] = 0x20;
		}
		else if(RepeatabilityStatus == LOW_REPEATABILITY)
		{
			Command[1] = 0x2B;
		}
		else
		{
			return;
		}

	}
	else if(Mps == MPS_4)
	{
		Command[0] = 0x23;
		if(RepeatabilityStatus == HIGH_REPEATABILITY)
		{
			Command[1] = 0x34;
		}
		else if(RepeatabilityStatus == MEDIUM_REPEATABILITY)
		{
			Command[1] = 0x22;
		}
		else if(RepeatabilityStatus == LOW_REPEATABILITY)
		{
			Command[1] = 0x29;
		}
		else
		{
			return;
		}

	}
	else if(Mps == MPS_10)
	{
		Command[0] = 0x27;
		if(RepeatabilityStatus == HIGH_REPEATABILITY)
		{
			Command[1] = 0x37;
		}
		else if(RepeatabilityStatus == MEDIUM_REPEATABILITY)
		{
			Command[1] = 0x21;
		}
		else if(RepeatabilityStatus == LOW_REPEATABILITY)
		{
			Command[1] = 0x2A;
		}
		else
		{
			return;
		}

	}
	else
	{
		return;
	}

	SHT30_Write_Command_2_Byte(Command);
	return;
}
void SHT30_Stop_Periodic_Acquisition_Mode()
{
	uint8_t Command[2] ={0x30,0x93};
	SHT30_Write_Command_2_Byte(Command);
}

uint8_t SHT30_Read_All_With_Periodic_Acquisition_Mode()
{
	uint8_t RawTempAndHum[6]={0};
	SHT30_Read_N_Byte(RawTempAndHum, 6);

	uint8_t crc1 = SHT30_CRC8_Calculator(&RawTempAndHum[0], 2);
	if(crc1 != RawTempAndHum[2])
	{
		return SHT30_ERR;
	}

	uint8_t crc2 = SHT30_CRC8_Calculator(&RawTempAndHum[3], 2);
	if(crc2 != RawTempAndHum[5])
	{
		return SHT30_ERR;
	}

	uint16_t raw_temp = (RawTempAndHum[0]<<8)|(RawTempAndHum[1]<<0);
	uint16_t raw_hum = (RawTempAndHum[3]<<8)|(RawTempAndHum[4]<<0);

	SHT30.Celcius = -45.0 + 175.0*raw_temp/65535.0;
	SHT30.Fahrenheit = -49.0 + 315.0*raw_temp/65535.0;
	SHT30.Kelvin = SHT30.Celcius + 273.15;
	SHT30.Humidity = 100.0*raw_hum/65535.0;

	return SHT30_OK;
}

uint8_t SHT30_Init(void *CommunicationHandleTypeDef)//
{
	hi2c = (I2C_HandleTypeDef *) CommunicationHandleTypeDef;
	if(SHT30_Check_Is_Device_Connected() == SHT30_ERR)
			return SHT30_ERR;

	//SHT30_Soft_Reset();
	SHT30.CHIP_ID =  SHT30_Read_Chip_Identification_Number();
	if(SHT30.CHIP_ID == 0)
		return SHT30_ERR;


	return SHT30_OK;
}

void SHT30_Soft_Reset()
{
	uint8_t Command[2] = {0x30,0xA2};
	SHT30_Write_Command_2_Byte(Command);
}
void SHT30_Enable_Heater()
{
	uint8_t Command[2] = {0x30,0x6D};
	SHT30_Write_Command_2_Byte(Command);
}
void SHT30_Disable_Heater()
{
	uint8_t Command[2] = {0x30,0x66};
	SHT30_Write_Command_2_Byte(Command);
}

uint8_t SHT30_Check_Is_Device_Connected()
{
		if(HAL_I2C_IsDeviceReady(hi2c,SHT30_I2C_ADDR1,1,100) == HAL_OK)
			SHT30.I2C_ADDR = SHT30_I2C_ADDR1;
		else if(HAL_I2C_IsDeviceReady(hi2c,SHT30_I2C_ADDR2,1,100) == HAL_OK)
			SHT30.I2C_ADDR = SHT30_I2C_ADDR2;
		else
			return SHT30_ERR;

		return SHT30_OK;

/*
	if(SHT30_Read_Chip_Identification_Number() == SHT30_CHIP_IDENTIFICATION_NUMBER)
			return SHT30_OK;
		else
			return SHT30_ERR;
	return SHT30_ERR;
*/
}


uint32_t SHT30_Read_Chip_Identification_Number()
{
	uint32_t CHIP_ID = 0;
	uint8_t Command[2]={0x37, 0x80};
	uint8_t ChipIDArr[6]={0};
	SHT30_Write_Command_2_Byte(Command);
	SHT30_Read_N_Byte(ChipIDArr, 6);
	uint8_t crc1 = SHT30_CRC8_Calculator(&ChipIDArr[0], 2);
	if(crc1 != ChipIDArr[2])
	{
		return 0;
	}

	uint8_t crc2 = SHT30_CRC8_Calculator(&ChipIDArr[3], 2);
	if(crc2 != ChipIDArr[5])
	{
		return 0;
	}

	CHIP_ID = (ChipIDArr[0]<<24)|(ChipIDArr[1]<<16)|(ChipIDArr[3]<<8)|(ChipIDArr[4]<<0);
	return CHIP_ID;
}


uint8_t SHT30_CRC8_Calculator(const uint8_t *idata, int len)
{
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t SHT30_VAL_CRC = 0xFF;

    for ( int8_t j = len; j; --j )
    {
    	SHT30_VAL_CRC ^= *idata++;
        for ( int8_t i = 8; i; --i )
        {
        	SHT30_VAL_CRC = ( SHT30_VAL_CRC & 0x80 ) ? (SHT30_VAL_CRC << 1) ^ POLYNOMIAL : (SHT30_VAL_CRC << 1);
        }
    }
    return SHT30_VAL_CRC;
}



uint32_t SHT30_Get_Chip_Identification_Number()
{
	return SHT30.CHIP_ID;
}


void SHT30_Write_Command_2_Byte(uint8_t *data)
{
	//HAL_I2C_Master_Transmit(hi2c, (SHT30_i2c_write_operation(SHT30.I2C_ADDR)), data, 2, 100);
	HAL_I2C_Master_Transmit(hi2c, SHT30.I2C_ADDR, data, 2, 100);
	return;
}
void SHT30_Read_N_Byte(uint8_t *data, uint8_t size)
{
	//HAL_I2C_Master_Receive(hi2c, (SHT30_i2c_read_operation(SHT30.I2C_ADDR)), data, size, 100);
	HAL_I2C_Master_Receive(hi2c, SHT30.I2C_ADDR, data, size, 1000);
	return;
}

double SHT30_Get_Temperature_In_Celcius()
{
	return SHT30.Celcius;
}
double SHT30_Get_Temperature_In_Fahrenheit()
{
	return SHT30.Fahrenheit;
}
double SHT30_Get_Temperature_In_Kelvin()
{
	return SHT30.Kelvin;
}
double SHT30_Get_Humidity()
{
	return SHT30.Humidity;
}


uint8_t SHT30_i2c_write_operation(uint8_t addr)
{
	return (addr&0b11111110);
}
uint8_t SHT30_i2c_read_operation(uint8_t addr)
{
	return (addr|0b00000001);
}
