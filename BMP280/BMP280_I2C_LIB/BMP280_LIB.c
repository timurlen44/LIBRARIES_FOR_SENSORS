/*
 * BMP280_LIB.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Timurleng44
 */
#include "math.h"
#include "BMP280_LIB.h"
#include "main.h"
static I2C_HandleTypeDef *hi2c;
uint8_t BMP280_Read_Register_1_Byte(uint8_t Reg_Addr);
uint16_t BMP280_Read_Register_2_Byte(uint8_t Reg_Addr);
uint32_t BMP280_Read_Register_3_Bytes(uint8_t Reg_Addr);
void BMP280_Read_Register_n_Byte(uint8_t Reg_Addr, uint8_t n, uint8_t *buff);
void BMP280_Write_Register_1_Byte(uint8_t Reg_Addr, uint8_t  Reg_Val);
void BMP280_Write_Register_2_Byte(uint8_t Reg_Addr, uint16_t  Reg_Val);
static void BMP280_Read_Trimming_Parameters();
static uint8_t BMP280_Read_Chip_Identification_Number();// Read chip ID from register of bmp280

void BMP280_Calculate_Compensate_Temperature();
void BMP280_Calculate_Compensate_Pressure();
typedef struct Trimming_Parameters_Struct{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

}Trimming_Parameters_Struct;

typedef struct Temprature_Parameters_Struct{
	int32_t adc_T;//raw temperature
	int32_t t_fine;//carries a fine resolution temperature value over to the pressure compensation formula.
	int32_t Compensate_Temperature;
	double celsius;
	double fahrenheit;
	double kelvin;
}Temprature_Parameters_Struct;

typedef struct Pressure_Parameters_Struct{
	int32_t adc_P;//raw pressure
	int32_t Compensate_Pressure;
	double Pa;//pressure in pascal
	double hPa;//pressure in hPa
	double hPa_sea_level;//pressure in sea level in hPa
	double Altitude;//Altitude
}Pressure_Parameters_Struct;

struct BMP280{
	uint16_t I2C_ADDR;
	uint8_t CHIP_ID;
	Trimming_Parameters_Struct Trimming_Parameters;
	Temprature_Parameters_Struct Temp;
	Pressure_Parameters_Struct  Press;
}BMP280;



uint8_t BMP280_Init(void *CommunicationHandleTypeDef,USE_CASE Use_Case)
{
	hi2c = (I2C_HandleTypeDef *) CommunicationHandleTypeDef;
	BMP280_Write_Register_1_Byte(reset, SOFT_RESET_COMMAND);
	while(1)
	{
		uint8_t s = BMP280_Read_Register_1_Byte(status);
		if((s & 1) == 0)
		{
			break;
		}
	}

	HAL_Delay(1000);
	if (BMP280_Check_Is_Device_Connected() == BMP280_OK)
	{

		if(Use_Case == Handheld_device_low_power_case)
		{
			BMP280_Set_Using_Settings(Normal_Mode, osrsp_x16, osrst_x2, IIR_F_C_4, t_sb_62_5 );
		}
		else if(Use_Case == Handheld_device_dynamic_case)
		{
			BMP280_Set_Using_Settings(Normal_Mode, osrsp_x4, osrst_x1, IIR_F_C_16, t_sb_0_5);
		}
		else if(Use_Case == Weather_monitoring_case)
		{
			BMP280_Set_Using_Settings(Forced_Mode, osrsp_x1, osrst_x1, IIR_F_C_Filter_Off, t_sb_1000);
		}
		else if(Use_Case == Elevator_floor_change_detection_case)
		{
			BMP280_Set_Using_Settings(Normal_Mode, osrsp_x4, osrst_x1, IIR_F_C_4, t_sb_125);
		}
		else if(Use_Case == Drop_detection_case)
		{
			BMP280_Set_Using_Settings(Normal_Mode, osrsp_x2, osrst_x1, IIR_F_C_Filter_Off, t_sb_0_5);
		}
		else if(Use_Case == Indoor_navigation_case)
		{
			BMP280_Set_Using_Settings(Normal_Mode, osrsp_x16, osrst_x2, IIR_F_C_16, t_sb_0_5);
		}
		else if(Use_Case == Custom_case)
		{
			//avoid
		}
		BMP280_Read_Trimming_Parameters();
		HAL_Delay(100);
		return BMP280_OK;
	}

	return BMP280_ERR;
}
uint8_t BMP280_Check_Is_Device_Connected()
{
		if(HAL_I2C_IsDeviceReady(hi2c,BMP280_I2C_ADDR1,1,100) == HAL_OK)
			BMP280.I2C_ADDR = BMP280_I2C_ADDR1;
		else if(HAL_I2C_IsDeviceReady(hi2c,BMP280_I2C_ADDR2,1,100) == HAL_OK)
			BMP280.I2C_ADDR = BMP280_I2C_ADDR2;
		else
			return BMP280_ERR;

		if(BMP280_Read_Chip_Identification_Number() == BMP280_CHIP_IDENTIFICATION_NUMBER)
				return BMP280_OK;
			else
				return BMP280_ERR;

}
static uint8_t BMP280_Read_Chip_Identification_Number()
{
	BMP280.CHIP_ID = BMP280_Read_Register_1_Byte(id);
	return BMP280.CHIP_ID;
}

uint8_t BMP280_Get_Chip_Identification_Number()
{
	return BMP280.CHIP_ID;
}

void BMP280_Set_Using_Settings(BMP280_MODE M,BMP280_OSRSP OSRS_P,BMP280_OSRST OSRS_T, BMP280_IIR_FILTER_COEFFICIENT IIR_F_C, BMP280_STANDBY_TIME T_SB)
{
	uint8_t ctrl_meas_reg_val = 0;
	uint8_t config_reg_val = 0;
	//Chose Mode
	if(M == Sleep_Mode)
	{
		ctrl_meas_reg_val |= 0b00000000;
	}
	else if(M == Forced_Mode)
	{
		ctrl_meas_reg_val |= 0b00000001; // or 0b00000010;
	}
	else if(M == Normal_Mode)
	{
		ctrl_meas_reg_val |= 0b00000011;

	}

	//Chose over-sampling rate of pressure
	if(OSRS_P == osrsp_x16)
	{
		ctrl_meas_reg_val |= 0b00010100;
	}
	else if(OSRS_P == osrsp_x8)
	{
		ctrl_meas_reg_val |= 0b00010000;
	}
	else if(OSRS_P == osrsp_x4)
	{
		ctrl_meas_reg_val |= 0b00001100;
	}
	else if(OSRS_P == osrsp_x2)
	{
		ctrl_meas_reg_val |= 0b00001000;
	}
	else if(OSRS_P == osrsp_x1)
	{
		ctrl_meas_reg_val |= 0b00000100;
	}

	//Chose over-sampling rate of temprature
	if(OSRS_T == osrst_x16)
	{
		ctrl_meas_reg_val |= 0b10100000;
	}
	else if(OSRS_T == osrst_x8)
	{
		ctrl_meas_reg_val |= 0b10000000;
	}
	else if(OSRS_T == osrst_x4)
	{
		ctrl_meas_reg_val |= 0b01100000;
	}
	else if(OSRS_T == osrst_x2)
	{
		ctrl_meas_reg_val |= 0b01000000;
	}
	else if(OSRS_T == osrst_x1)
	{
		ctrl_meas_reg_val |= 0b00100000;
	}

	//Chose IIR filter coefficient
	if(IIR_F_C == IIR_F_C_16)
	{
		config_reg_val |= 0b00010000;
	}
	else if(IIR_F_C == IIR_F_C_8)
	{
		config_reg_val |= 0b00001100;
	}
	else if(IIR_F_C == IIR_F_C_4)
	{
		config_reg_val |= 0b00001000;
	}
	else if(IIR_F_C == IIR_F_C_2)
	{
		config_reg_val |= 0b00000100;
	}
	else if(IIR_F_C == IIR_F_C_Filter_Off)
	{
		config_reg_val |= 0b00000000;
	}

	//Chose standby time
	if(T_SB == t_sb_0_5)
	{
		config_reg_val |= 0b00000000;
	}
	else if(T_SB == t_sb_62_5)
	{
		config_reg_val |= 0b00100000;
	}
	else if(T_SB == t_sb_125)
	{
		config_reg_val |= 0b01000000;
	}
	else if(T_SB == t_sb_250)
	{
		config_reg_val |= 0b01100000;
	}
	else if(T_SB == t_sb_500)
	{
		config_reg_val |= 0b10000000;
	}
	else if(T_SB == t_sb_1000)
	{
		config_reg_val |= 0b10100000;
	}
	else if(T_SB == t_sb_2000)
	{
		config_reg_val |= 0b11000000;
	}
	else if(T_SB == t_sb_4000)
	{
		config_reg_val |= 0b11100000;
	}

	BMP280_Write_Register_1_Byte(config, config_reg_val);
	BMP280_Write_Register_1_Byte(ctrl_meas, ctrl_meas_reg_val);

	return;
}

static void BMP280_Read_Trimming_Parameters()
{
	uint8_t buff[24];
	BMP280_Read_Register_n_Byte(dig_T1_Addr, 24, buff);
	BMP280.Trimming_Parameters.dig_T1 =  ((buff[1]<<8 ) | buff[0]);
	BMP280.Trimming_Parameters.dig_T2 =  ((buff[3]<<8 ) | buff[2]);
	BMP280.Trimming_Parameters.dig_T3 =  ((buff[5]<<8 ) | buff[4]);
	BMP280.Trimming_Parameters.dig_P1 =  ((buff[7]<<8 ) | buff[6]);
	BMP280.Trimming_Parameters.dig_P2 =  ((buff[9]<<8 ) | buff[8]);
	BMP280.Trimming_Parameters.dig_P3 =  ((buff[11]<<8) | buff[10]);
	BMP280.Trimming_Parameters.dig_P4 =  ((buff[13]<<8) | buff[12]);
	BMP280.Trimming_Parameters.dig_P5 =  ((buff[15]<<8) | buff[14]);
	BMP280.Trimming_Parameters.dig_P6 =  ((buff[17]<<8) | buff[16]);
	BMP280.Trimming_Parameters.dig_P7 =  ((buff[19]<<8) | buff[18]);
	BMP280.Trimming_Parameters.dig_P8 =  ((buff[21]<<8) | buff[20]);
	BMP280.Trimming_Parameters.dig_P9 =  ((buff[23]<<8) | buff[22]);

	return;
}

uint8_t BMP280_Read_Register_1_Byte(uint8_t Reg_Addr)
{
	uint8_t Reg_Val = 0;
	HAL_I2C_Mem_Read(hi2c, BMP280.I2C_ADDR, Reg_Addr, 1, &Reg_Val, 1, 100);
	return Reg_Val;
}

uint16_t BMP280_Read_Register_2_Byte(uint8_t Reg_Addr)
{
	uint16_t Reg_Val = 0;
	HAL_I2C_Mem_Read(hi2c, BMP280.I2C_ADDR, Reg_Addr, 1, (uint8_t *)&Reg_Val, 2, 100);
	return Reg_Val;
}

uint32_t BMP280_Read_Register_3_Bytes(uint8_t Reg_Addr)
{
	uint32_t Reg_Val = 0;
	uint8_t data[3]={0};
	HAL_I2C_Mem_Read(hi2c, BMP280.I2C_ADDR, Reg_Addr, 1, data, 3, HAL_MAX_DELAY);
	Reg_Val = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	return Reg_Val;
}

void BMP280_Read_Register_n_Byte(uint8_t Reg_Addr, uint8_t n, uint8_t *buff)
{
	HAL_I2C_Mem_Read(hi2c,BMP280.I2C_ADDR , Reg_Addr, 1, &buff[0], n, 100);
	return;
}

void BMP280_Write_Register_1_Byte(uint8_t Reg_Addr, uint8_t  Reg_Val)
{
	HAL_I2C_Mem_Write(hi2c, BMP280.I2C_ADDR,Reg_Addr,1,&Reg_Val,1,100);
	return;
}
void BMP280_Write_Register_2_Byte(uint8_t Reg_Addr, uint16_t  Reg_Val)
{
	HAL_I2C_Mem_Write(hi2c, BMP280.I2C_ADDR, Reg_Addr,1, (uint8_t *)&Reg_Val, 2, 100);
	return;
}

void BMP280_Process_All(void);
void BMP280_Read_All(void);
void BMP280_Read_Raw_Temperature(void);
void BMP280_Read_Raw_Pressure(void);
void BMP280_Burst_Read_Raw_Temperature_And_Raw_Pressure(void);

void BMP280_Read_Raw_Temperature(void)
{
	uint32_t adc_T = BMP280_Read_Register_3_Bytes(0xFA);
	BMP280.Temp.adc_T = adc_T;
	return;
}

void BMP280_Read_Raw_Pressure(void)
{
	uint32_t adc_P = BMP280_Read_Register_3_Bytes(0xF7);
	BMP280.Press.adc_P = adc_P;
	return;
}
void BMP280_Burst_Read_Raw_Temperature_And_Raw_Pressure(void)
{
	uint32_t adc_P    = 0;
	uint32_t adc_T = 0;
	uint8_t data[6]={0};
	BMP280_Read_Register_n_Byte(press_msb, 6, data);
	adc_P = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_T = data[3] << 12 | data[4] << 4 | data[5] >> 4;
	BMP280.Press.adc_P = adc_P;
	BMP280.Temp.adc_T  = adc_T;
	return;
}

void BMP280_Calculate_Compensate_Temperature()
{
	int32_t var1, var2, Compensate_Temp;
	uint16_t dig_T1 = BMP280.Trimming_Parameters.dig_T1;
	int16_t  dig_T2 = BMP280.Trimming_Parameters.dig_T2;
	int16_t  dig_T3 = BMP280.Trimming_Parameters.dig_T3;
	int32_t t_fine = BMP280.Temp.t_fine;
	int32_t adc_T = BMP280.Temp.adc_T;

	  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
	          ((int32_t)dig_T2)) >>
	         11;

	  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
	            ((adc_T >> 4) - ((int32_t)dig_T1))) >>
	           12) *
	          ((int32_t)dig_T3)) >>
	         14;

	t_fine = var1 + var2;
	BMP280.Temp.t_fine = t_fine;
	Compensate_Temp = (t_fine * 5 + 128) >> 8;
	BMP280.Temp.Compensate_Temperature = Compensate_Temp;
	return;
}

void BMP280_Calculate_Compensate_Pressure()
{
	int64_t var1, var2, Compensate_Press;
	uint16_t dig_P1 = BMP280.Trimming_Parameters.dig_P1;
	int16_t dig_P2 = BMP280.Trimming_Parameters.dig_P2;
	int16_t dig_P3 = BMP280.Trimming_Parameters.dig_P3;
	int16_t dig_P4 = BMP280.Trimming_Parameters.dig_P4;
	int16_t dig_P5 = BMP280.Trimming_Parameters.dig_P5;
	int16_t dig_P6 = BMP280.Trimming_Parameters.dig_P6;
	int16_t dig_P7 = BMP280.Trimming_Parameters.dig_P7;
	int16_t dig_P8 = BMP280.Trimming_Parameters.dig_P8;
	int16_t dig_P9 = BMP280.Trimming_Parameters.dig_P9;
	var1 = (int64_t) BMP280.Temp.t_fine - 128000;
	var2 = var1 * var1 * (int64_t) dig_P6;
	var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
	var2 = var2 + (((int64_t) dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8)
				+ ((var1 * (int64_t) dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dig_P1) >> 33;

	if (var1 == 0) {
		Compensate_Press = 0;
		BMP280.Press.Compensate_Pressure = Compensate_Press;
		return;  // avoid exception caused by division by zero
	}

	Compensate_Press = 1048576 - BMP280.Press.adc_P;
	Compensate_Press = (((Compensate_Press << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dig_P9 * (Compensate_Press >> 13) * (Compensate_Press >> 13)) >> 25;
	var2 = ((int64_t) dig_P8 * Compensate_Press) >> 19;
	Compensate_Press = ((Compensate_Press + var1 + var2) >> 8) + ((int64_t) dig_P7 << 4);
	BMP280.Press.Compensate_Pressure = Compensate_Press;

	return ;
}

void BMP280_Read_All(void)
{
	uint8_t status_reg_val = BMP280_Read_Register_1_Byte(status);

	if(status_reg_val)
	{
		BMP280_Burst_Read_Raw_Temperature_And_Raw_Pressure();
		BMP280_Process_All();
	}
	return;
}
void BMP280_Process_All(void) // Process raw pressure and raw temperature.
{

	BMP280_Calculate_Compensate_Temperature();
	BMP280_Calculate_Compensate_Pressure();
	BMP280.Temp.celsius = BMP280.Temp.Compensate_Temperature/100.0;
	BMP280.Temp.kelvin = BMP280.Temp.celsius + 273.15;
	BMP280.Temp.fahrenheit = ((BMP280.Temp.celsius*1.8)+32.0);

	BMP280.Press.Pa = BMP280.Press.Compensate_Pressure/256.0;
	BMP280.Press.hPa = BMP280.Press.Pa*0.01;
	BMP280.Press.Altitude = 44330 * (1.0 - pow(BMP280.Press.hPa / BMP280.Press.hPa_sea_level, 0.1903));


	return;
}











double BMP280_Get_Temperature_In_Celcius()
{
	return BMP280.Temp.celsius;
}
double BMP280_Get_Temperature_In_Kelvin()
{
	return BMP280.Temp.kelvin;
}
double BMP280_Get_Temperature_In_Fahrenheit()
{
	return BMP280.Temp.fahrenheit;
}

double BMP280_Get_Pressure_In_Pa()
{
	return BMP280.Press.Pa;
}
double BMP280_Get_Temperature_In_hPa()
{
	return BMP280.Press.hPa;
}

double BMP280_Get_Altitude()
{
	return BMP280.Press.Altitude;
}
double BMP280_Set_Pressure_In_Sea_Level(double hPa_in_sea_level)
{
	return BMP280.Press.hPa_sea_level = hPa_in_sea_level;
}
