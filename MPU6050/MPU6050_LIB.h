/*
 * MPU6050_LIB2.h
 *
 *  Created on: 19 Mar 2023
 *      Author: Timurleng44
 */

#ifndef INC_MPU6050_LIB_H_
#define INC_MPU6050_LIB_H_
#include "main.h"
#include "math.h"
//MPU6050 ADDRESS
#define MPU6050_ADDR 0x68 << 1

//MPU6050 REGISTERS
#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75


#define MPU6050_OK 1
#define MPU6050_ERROR 0

#define DATA_IS_READY 1
#define DATA_IS_NOT_READY 0

#define PI 3.14159265359

typedef enum FS_SEL
{
	FS_SEL_250 =  (uint8_t)(0<<3),
	FS_SEL_500 =  (uint8_t)(1<<3),
	FS_SEL_1000 = (uint8_t)(2<<3),
	FS_SEL_2000 = (uint8_t)(3<<3)
}FS_SEL_ENUM_TYPEDEF;

typedef enum AFS_SEL
{
	AFS_SEL_2g =  (uint8_t)(0<<3),
	AFS_SEL_4g =  (uint8_t)(1<<3),
	AFS_SEL_8g =  (uint8_t)(2<<3),
	AFS_SEL_16g = (uint8_t)(3<<3)
}AFS_SEL_ENUM_TYPEDEF;

uint8_t MPU6050_INIT(uint16_t MPU6050_Address, uint32_t I2C_TimeOut, FS_SEL_ENUM_TYPEDEF FS_SEL, AFS_SEL_ENUM_TYPEDEF AFS_SEL);
uint8_t MPU6050_IS_DEVICE_OK();
uint8_t MPU6050_READ_WHO_AM_I_REGISTER();
void MPU6050_INTERRUPT_ENABLE();
void MPU6050_When_The_Last_Interrupt_Occurred();
void MPU6050_Elapsed_Time_Calculator();
void MPU6050_Set_Offsets(double acc_offset_x, double acc_offset_y, double acc_offset_z, double gyro_offset_x, double gyro_offset_y, double gyro_offset_z);
void MPU6050_Calculate_Offsets(double* acc_offset_x, double* acc_offset_y, double* acc_offset_z, double* gyro_offset_x, double* gyro_offset_y, double* gyro_offset_z);

void MPU6050_Reset();
uint8_t MPU6050_Reset_Status();
uint8_t MPU6050_GET_INTERRUPT_STATE();
void MPU6050_SET_INTERRUPT_STATE();
void MPU6050_RESET_INTERRUPT_STATE();

uint8_t MPU6050_Read_Register_1_Byte(uint8_t Reg_Addr);
void MPU6050_Write_Register_1_Byte(uint8_t Reg_Addr, uint8_t  Reg_Val);

void MPU6050_Update_All(void);
void MPU6050_Process_All_Data(void);
double MPU6050_Get_Temperature_In_Kelvin();
double MPU6050_Get_Temperature_In_Fahrenheit();
double MPU6050_Get_Temperature_In_Celcius();

int16_t MPU6050_Get_Raw_Acc_X();
int16_t MPU6050_Get_Raw_Acc_Y();
int16_t MPU6050_Get_Raw_Acc_Z();

int16_t MPU6050_Get_Raw_Gyro_X();
int16_t MPU6050_Get_Raw_Gyro_Y();
int16_t MPU6050_Get_Raw_Gyro_Z();

double MPU6050_Get_Acc_X();
double MPU6050_Get_Acc_Y();
double MPU6050_Get_Acc_Z();

double MPU6050_Get_Gyro_X();
double MPU6050_Get_Gyro_Y();
double MPU6050_Get_Gyro_Z();

double MPU6050_Get_Roll();
double MPU6050_Get_Pitch();
double MPU6050_Get_Yaw();

double MPU6050_Get_Roll_With_Complemantary_Filter();
double MPU6050_Get_Pitch_With_Complemantary_Filter();
double MPU6050_Get_Yaw_With_Complemantary_Filter();

double MPU6050_Get_Roll_With_Low_Pass_Filter();
double MPU6050_Get_Pitch_With_Low_Pass_Filter();
double MPU6050_Get_Yaw_With_Low_Pass_Filter();


void MPU6050_Set_Low_Pass_Filter_Coefficient(double MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA, double MULTIPLE_COEFFICIENT_OF_PRESENT_DATA);
void MPU6050_Set_Complemantary_Filter_Coefficients(double GYRO_COEFFICIENT, double ACC_COEFFICIENT);





#endif /* INC_MPU6050_LIB_H_ */
