/*
 * MPU6050_LIB2.c
 *
 *  Created on: 19 Mar 2023
 *      Author: Timurleng44
 */


#include "MPU6050_LIB.h"
extern I2C_HandleTypeDef hi2c1;
void MPU6050_Reset()
{
	MPU6050_Write_Register_1_Byte(PWR_MGMT_1, 0b10000000);
	HAL_Delay(100);
	while(MPU6050_Reset_Status() != MPU6050_OK);
	//Reset completed
}

uint8_t MPU6050_Reset_Status()
{
	uint8_t val = (MPU6050_Read_Register_1_Byte(PWR_MGMT_1) & 0b10000000);
	if(val == 0){
		return MPU6050_OK;
	}
	else{
		return MPU6050_ERROR;
	}
}

typedef struct MPU6050_RAW_DATA_STRUCT
{
	int16_t x;
	int16_t y;
	int16_t z;
}MPU6050_RAW_DATA_TYPDEF;

typedef struct MPU6050_DATA_STRUCT
{
	double x;
	double y;
	double z;
}MPU6050_DATA_TYPDEF;

typedef struct MPU6050_EULER_ANGLES_DATA_STRUCT
{
	double roll;
	double pitch;
	double yaw;
}MPU6050_EULER_ANGLES_DATA_TYPDEF;

typedef struct MPU6050_TEMP_DATA_STRUCT
{
	int16_t raw_temp_data;
	double celsius;
	double fahrenheit;
	double kelvin;
}MPU6050_TEMP_DATA_TYPEDEF;

typedef struct MPU6050_LOW_PASS_FILTER_SETTINGS_STRUCT
{
	double MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA;
	double MULTIPLE_COEFFICIENT_OF_PRESENT_DATA;
}MPU6050_LOW_PASS_FILTER_SETTINGS_TYPEDEF;


typedef struct MPU6050_COMPLEMANTARY_FILTER_SETTINGS_STRUCT
{
	double GYRO_COEFFICIENT;
	double ACC_COEFFICIENT;
}MPU6050_COMPLEMANTARY_FILTER_SETTINGS_TYPEDEF;

typedef struct MPU6050_SENSOR_DATA_STRUCT
{
	MPU6050_TEMP_DATA_TYPEDEF temp;
	MPU6050_DATA_TYPDEF acc;// accelerometer raw datas
	MPU6050_DATA_TYPDEF gyro;// gyroscope raw datas
	MPU6050_RAW_DATA_TYPDEF raw_gyro;// gyroscope raw datas
	MPU6050_RAW_DATA_TYPDEF raw_acc;// gyroscope raw datas

	MPU6050_DATA_TYPDEF acc_offset;
	MPU6050_DATA_TYPDEF gyro_offset;

	MPU6050_EULER_ANGLES_DATA_TYPDEF euler_angles;
	MPU6050_EULER_ANGLES_DATA_TYPDEF euler_angles_with_gyro;

	MPU6050_LOW_PASS_FILTER_SETTINGS_TYPEDEF low_pass_filter_settings;
	MPU6050_EULER_ANGLES_DATA_TYPDEF euler_angles_with_low_pass_filter;

	MPU6050_COMPLEMANTARY_FILTER_SETTINGS_TYPEDEF complemantary_filter_settings;
	MPU6050_EULER_ANGLES_DATA_TYPDEF euler_angles_with_complemantary_filter;

	uint32_t when_the_last_interrupt_occurred_in_ms;
	uint32_t previous_time;
	uint32_t current_time;
	double elapsed_time_in_second;
	uint8_t MPU6050_IS_DATA_READY;

	double ACCELEROMETER_LSB_SENSIVITY;
	double GYROSCOPE_LSB_SENSIVITY;
}MPU6050_SENSOR_DATA_TYPDEF;
MPU6050_SENSOR_DATA_TYPDEF mpu6050_data;

typedef struct MPU6050_Communication_Settings_Struct
{
	I2C_HandleTypeDef *hi2c;
	uint16_t MPU6050_Address;
	uint32_t I2C_Timeout;

	double ACCELEROMETER_LSB_SENSIVITY;
	double GYROSCOPE_LSB_SENSIVITY;

}MPU6050_Communication_Settings_Struct;

MPU6050_Communication_Settings_Struct MPU6050;



uint8_t MPU6050_INIT(uint16_t MPU6050_Address, uint32_t I2C_TimeOut, FS_SEL_ENUM_TYPEDEF FS_SEL, AFS_SEL_ENUM_TYPEDEF AFS_SEL)
{
	//MPU6050.hi2c = hi2c;
	//MPU6050.hi2c = &hi2c1;
	MPU6050.MPU6050_Address = MPU6050_Address;
	MPU6050.I2C_Timeout = I2C_TimeOut;

	MPU6050_Reset();
	MPU6050_Write_Register_1_Byte(PWR_MGMT_1, 0x00);
	MPU6050_Write_Register_1_Byte(SMPLRT_DIV, 0x07);
	MPU6050_Write_Register_1_Byte(GYRO_CONFIG, FS_SEL);
	MPU6050_Write_Register_1_Byte(ACCEL_CONFIG, AFS_SEL);

	switch (FS_SEL)
	{
		case FS_SEL_250:
			MPU6050.GYROSCOPE_LSB_SENSIVITY = 131.0;// LSB/°/s
			break;

		case FS_SEL_500:
			MPU6050.GYROSCOPE_LSB_SENSIVITY = 65.5;// LSB/°/s
			break;

		case FS_SEL_1000:
			MPU6050.GYROSCOPE_LSB_SENSIVITY = 32.8;// LSB/°/s
			break;

		case FS_SEL_2000:
			MPU6050.GYROSCOPE_LSB_SENSIVITY = 16.4;// LSB/°/s
			break;
	}

	switch (AFS_SEL)
	{
		case AFS_SEL_2g:
			MPU6050.ACCELEROMETER_LSB_SENSIVITY = 16384.0;// LSB/g
			break;

		case AFS_SEL_4g:
			MPU6050.ACCELEROMETER_LSB_SENSIVITY = 8192.0;// LSB/g
			break;

		case AFS_SEL_8g:
			MPU6050.ACCELEROMETER_LSB_SENSIVITY = 4096.0;// LSB/g
			break;

		case AFS_SEL_16g:
			MPU6050.ACCELEROMETER_LSB_SENSIVITY = 2048.0;// LSB/g
			break;
	}
	mpu6050_data.current_time = HAL_GetTick();
	return MPU6050_IS_DEVICE_OK();
}

void MPU6050_INTERRUPT_ENABLE()
{
	MPU6050_Write_Register_1_Byte(INT_PIN_CFG, 0b00010000);
	MPU6050_Write_Register_1_Byte(INT_ENABLE, 0x01);
	//MPU6050_When_The_Last_Interrupt_Occurred();
	return;
}


uint8_t MPU6050_GET_INTERRUPT_STATE()
{
	return mpu6050_data.MPU6050_IS_DATA_READY;
}

void MPU6050_SET_INTERRUPT_STATE()
{
	mpu6050_data.MPU6050_IS_DATA_READY = DATA_IS_READY;
	mpu6050_data.current_time = HAL_GetTick();
	//MPU6050_Elapsed_Time_Calculator();
	//MPU6050_When_The_Last_Interrupt_Occurred();
	return;
}
void MPU6050_RESET_INTERRUPT_STATE()
{
	mpu6050_data.MPU6050_IS_DATA_READY = DATA_IS_NOT_READY;
	return;
}


uint8_t MPU6050_IS_DEVICE_OK()
{
	if((MPU6050_READ_WHO_AM_I_REGISTER() == (MPU6050.MPU6050_Address>>1))){
		return MPU6050_OK;
	}
	else{
		return MPU6050_ERROR;
	}
}


uint8_t MPU6050_READ_WHO_AM_I_REGISTER()
{
	return MPU6050_Read_Register_1_Byte(WHO_AM_I);
}



uint8_t MPU6050_Read_Register_1_Byte(uint8_t Reg_Addr)
{
	uint8_t Reg_Val = 0;
	//HAL_I2C_Mem_Read(MPU6050.hi2c, MPU6050.MPU6050_Address, Reg_Addr, 1, &Reg_Val, 1, MPU6050.I2C_Timeout);
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, Reg_Addr, 1, &Reg_Val, 1, 100);
	return Reg_Val;
}

void MPU6050_Write_Register_1_Byte(uint8_t Reg_Addr, uint8_t  Reg_Val)
{
	//HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050.MPU6050_Address,Reg_Addr,1,&Reg_Val,1,MPU6050.I2C_Timeout);
	HAL_I2C_Mem_Write(&hi2c1, 0xD0,Reg_Addr,1,&Reg_Val,1,100);
}

void MPU6050_Set_Offsets(double acc_offset_x, double acc_offset_y, double acc_offset_z, double gyro_offset_x, double gyro_offset_y, double gyro_offset_z)
{
	mpu6050_data.acc_offset.x = acc_offset_x;
	mpu6050_data.acc_offset.y = acc_offset_y;
	mpu6050_data.acc_offset.z = acc_offset_z;

	mpu6050_data.gyro_offset.x = gyro_offset_x;
	mpu6050_data.gyro_offset.y = gyro_offset_y;
	mpu6050_data.gyro_offset.z = gyro_offset_z;

	return;
}

void MPU6050_Calculate_Offsets(double* acc_offset_x, double* acc_offset_y, double* acc_offset_z, double* gyro_offset_x, double* gyro_offset_y, double* gyro_offset_z)
{
	int16_t counter = 0;

		while(1)
		{


				counter++;
				MPU6050_Update_All();
				MPU6050_Process_All_Data();

				(*acc_offset_x) += mpu6050_data.acc.x;
				(*acc_offset_y) += mpu6050_data.acc.y;
				(*acc_offset_z) += mpu6050_data.acc.z;


				(*gyro_offset_x) += mpu6050_data.gyro.x;
				(*gyro_offset_y) += mpu6050_data.gyro.y;
				(*gyro_offset_z) += mpu6050_data.gyro.z;
				if(counter == 200)
				{

					(*acc_offset_x) /= 200.0;
					(*acc_offset_y) /= 200.0;
					(*acc_offset_z) /= 200.0;
					(*acc_offset_z) = (*acc_offset_z) - 1.00;

					(*gyro_offset_x) /= 200.0;
					(*gyro_offset_y) /= 200.0;
					(*gyro_offset_z) /= 200.0;
					return;
					//break;
			    }
		}
/*
		(*acc_offset_x) /= 200.0;
		(*acc_offset_y) /= 200.0;
		(*acc_offset_z) /= 200.0;
		(*acc_offset_z) = (*acc_offset_z) - 1.00;

		(*gyro_offset_x) /= 200.0;
		(*gyro_offset_y) /= 200.0;
		(*gyro_offset_z) /= 200.0;
*/
		/*

		gyro_offs_x = gyro_offs_x/200;
		gyro_offs_y = gyro_offs_y/200;
		gyro_offs_z = gyro_offs_z/200;
		*/
		return;

}


double MPU6050_Get_Temperature_In_Celcius()
{
	return mpu6050_data.temp.celsius;
}
double MPU6050_Get_Temperature_In_Kelvin()
{
	return mpu6050_data.temp.kelvin;
}
double MPU6050_Get_Temperature_In_Fahrenheit()
{
	return mpu6050_data.temp.fahrenheit;
}

int16_t MPU6050_Get_Raw_Acc_X()
{
	return mpu6050_data.raw_acc.x;
}
int16_t MPU6050_Get_Raw_Acc_Y()
{
	return mpu6050_data.raw_acc.y;
}
int16_t MPU6050_Get_Raw_Acc_Z()
{
	return mpu6050_data.raw_acc.z;
}

int16_t MPU6050_Get_Raw_Gyro_X()
{
	return mpu6050_data.raw_gyro.x;
}
int16_t MPU6050_Get_Raw_Gyro_Y()
{
	return mpu6050_data.raw_gyro.x;
}
int16_t MPU6050_Get_Raw_Gyro_Z()
{
	return mpu6050_data.raw_gyro.x;
}

double MPU6050_Get_Acc_X()
{
	return mpu6050_data.acc.x;
}
double MPU6050_Get_Acc_Y()
{
	return mpu6050_data.acc.y;
}
double MPU6050_Get_Acc_Z()
{
	return mpu6050_data.acc.z;
}

double MPU6050_Get_Gyro_X()
{
	return mpu6050_data.gyro.x;
}
double MPU6050_Get_Gyro_Y()
{
	return mpu6050_data.gyro.y;
}
double MPU6050_Get_Gyro_Z()
{
	return mpu6050_data.gyro.z;
}


double MPU6050_Get_Roll()
{
	return mpu6050_data.euler_angles.roll;
}
double MPU6050_Get_Pitch()
{
	return mpu6050_data.euler_angles.pitch;
}
double MPU6050_Get_Yaw()
{
	return mpu6050_data.euler_angles.yaw;
}

double MPU6050_Get_Roll_With_Complemantary_Filter()
{
	return mpu6050_data.euler_angles_with_complemantary_filter.roll;
}
double MPU6050_Get_Pitch_With_Complemantary_Filter()
{
	return mpu6050_data.euler_angles_with_complemantary_filter.pitch;
}
double MPU6050_Get_Yaw_With_Complemantary_Filter()
{
	return mpu6050_data.euler_angles_with_complemantary_filter.yaw;
}

double MPU6050_Get_Roll_With_Low_Pass_Filter()
{
	return mpu6050_data.euler_angles_with_low_pass_filter.roll;
}
double MPU6050_Get_Pitch_With_Low_Pass_Filter()
{
	return mpu6050_data.euler_angles_with_low_pass_filter.pitch;
}
double MPU6050_Get_Yaw_With_Low_Pass_Filter()
{
	return mpu6050_data.euler_angles_with_low_pass_filter.yaw;
}


uint8_t buffer[14] = {0};
void MPU6050_Update_All()
{
	mpu6050_data.previous_time = mpu6050_data.current_time;
	mpu6050_data.current_time = HAL_GetTick();
	mpu6050_data.elapsed_time_in_second = ((mpu6050_data.current_time - mpu6050_data.previous_time) / 1000.0);
	//if(HAL_I2C_Mem_Read(MPU6050.hi2c, MPU6050.MPU6050_Address, ACCEL_XOUT_H, 1, buffer, 14, MPU6050.I2C_Timeout) != HAL_OK)
	if(HAL_I2C_Mem_Read(&hi2c1, MPU6050.MPU6050_Address, ACCEL_XOUT_H, 1, buffer, 14, MPU6050.I2C_Timeout) != HAL_OK)
	{
		return;
	}

	mpu6050_data.raw_acc.x = (buffer[0]<<8)|(buffer[1]);
	mpu6050_data.raw_acc.y = (buffer[2]<<8)|(buffer[3]);
	mpu6050_data.raw_acc.z = (buffer[4]<<8)|(buffer[5]);
	mpu6050_data.temp.raw_temp_data = (buffer[6]<<8)|(buffer[7]);
	mpu6050_data.raw_gyro.x = (buffer[8]<<8)|(buffer[9]);
	mpu6050_data.raw_gyro.y = (buffer[10]<<8)|(buffer[11]);
	mpu6050_data.raw_gyro.z = (buffer[12]<<8)|(buffer[13]);

/*
	for(int i = 0;i<14;i++)
	{
		buffer[i] = '\0';
	}
	HAL_I2C_Mem_Read(&hi2c1, MPU6050.MPU6050_Address, GYRO_XOUT_H, 1, &buffer[0], 6, MPU6050.I2C_Timeout);
	mpu6050_data.current_time = HAL_GetTick();
	mpu6050_data.elapsed_time_in_second = ((mpu6050_data.current_time - mpu6050_data.previous_time) / 1000.0);
	mpu6050_data.raw_gyro.x = (buffer[0]<<8)|(buffer[1]);
	mpu6050_data.raw_gyro.y = (buffer[2]<<8)|(buffer[3]);
	mpu6050_data.raw_gyro.z = (buffer[4]<<8)|(buffer[5]);
	*/
}


void MPU6050_Process_All_Data()
{

	//Raw Temparature is converting to celcius, kelvin and fahrenheit.
	mpu6050_data.temp.celsius = ((mpu6050_data.temp.raw_temp_data/340.0) + 36.53);
	mpu6050_data.temp.kelvin = mpu6050_data.temp.celsius+273.15;
	mpu6050_data.temp.fahrenheit = ((mpu6050_data.temp.celsius*1.8)+32.0);

	//Acceleration is calculating as g type
	mpu6050_data.acc.x = (double)mpu6050_data.raw_acc.x / MPU6050.ACCELEROMETER_LSB_SENSIVITY;
	mpu6050_data.acc.y = (double)mpu6050_data.raw_acc.y / MPU6050.ACCELEROMETER_LSB_SENSIVITY;
	mpu6050_data.acc.z = (double)mpu6050_data.raw_acc.z / MPU6050.ACCELEROMETER_LSB_SENSIVITY;

	//Acceleration offset
	mpu6050_data.acc.x -= mpu6050_data.acc_offset.x;
	mpu6050_data.acc.y -= mpu6050_data.acc_offset.y;
	mpu6050_data.acc.z -= mpu6050_data.acc_offset.z;

	//Gyroscope is calculating
	mpu6050_data.gyro.x = (double)mpu6050_data.raw_gyro.x / MPU6050.GYROSCOPE_LSB_SENSIVITY;
	mpu6050_data.gyro.y = (double)mpu6050_data.raw_gyro.y / MPU6050.GYROSCOPE_LSB_SENSIVITY;
	mpu6050_data.gyro.z = (double)mpu6050_data.raw_gyro.z / MPU6050.GYROSCOPE_LSB_SENSIVITY;

	//Gyroscope offset
	mpu6050_data.gyro.x -= mpu6050_data.gyro_offset.x;
	mpu6050_data.gyro.y -= mpu6050_data.gyro_offset.y;
	mpu6050_data.gyro.z -= mpu6050_data.gyro_offset.z;

	//Euler angles are calculating with accelerometer
	mpu6050_data.euler_angles.roll  = atan(mpu6050_data.acc.y / sqrt(pow(mpu6050_data.acc.x, 2) + pow(mpu6050_data.acc.z, 2))) * 180 / PI;
	mpu6050_data.euler_angles.pitch = atan(-1 * mpu6050_data.acc.x / sqrt(pow(mpu6050_data.acc.y , 2) + pow(mpu6050_data.acc.z, 2))) * 180 / PI;
	mpu6050_data.euler_angles.yaw   = 0.00;

	//Euler angles are calculating with gyroscope (angular velocity * elapsed time = angle)
	mpu6050_data.euler_angles_with_gyro.roll += mpu6050_data.gyro.x * mpu6050_data.elapsed_time_in_second;
	mpu6050_data.euler_angles_with_gyro.pitch += mpu6050_data.gyro.y * mpu6050_data.elapsed_time_in_second;
	mpu6050_data.euler_angles_with_gyro.yaw  += mpu6050_data.gyro.z * mpu6050_data.elapsed_time_in_second;

	//Low pass filter is implemented to euler angles
	mpu6050_data.euler_angles_with_low_pass_filter.roll  = mpu6050_data.euler_angles_with_low_pass_filter.roll  * mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA + mpu6050_data.euler_angles.roll  * mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PRESENT_DATA;
	mpu6050_data.euler_angles_with_low_pass_filter.pitch = mpu6050_data.euler_angles_with_low_pass_filter.pitch * mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA + mpu6050_data.euler_angles.pitch * mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PRESENT_DATA;
	mpu6050_data.euler_angles_with_low_pass_filter.yaw   = mpu6050_data.euler_angles_with_low_pass_filter.yaw   * mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA + mpu6050_data.euler_angles.yaw   * mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PRESENT_DATA;

	//Complemantary filter is implemented to euler angles
	mpu6050_data.euler_angles_with_complemantary_filter.roll  = (mpu6050_data.euler_angles_with_complemantary_filter.roll  + mpu6050_data.gyro.x * mpu6050_data.elapsed_time_in_second)*0.8 + (mpu6050_data.euler_angles.roll )*0.2;
	mpu6050_data.euler_angles_with_complemantary_filter.pitch = (mpu6050_data.euler_angles_with_complemantary_filter.pitch + mpu6050_data.gyro.y * mpu6050_data.elapsed_time_in_second)*0.8 + (mpu6050_data.euler_angles.pitch)*0.2;
	mpu6050_data.euler_angles_with_complemantary_filter.yaw   = 0;

	return;
}


void MPU6050_Set_Low_Pass_Filter_Coefficient(double MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA, double MULTIPLE_COEFFICIENT_OF_PRESENT_DATA)
{
	mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA = MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA;
	mpu6050_data.low_pass_filter_settings.MULTIPLE_COEFFICIENT_OF_PRESENT_DATA = MULTIPLE_COEFFICIENT_OF_PRESENT_DATA;
	return;
}

void MPU6050_Set_Complemantary_Filter_Coefficients(double GYRO_COEFFICIENT, double ACC_COEFFICIENT)
{
	mpu6050_data.complemantary_filter_settings.GYRO_COEFFICIENT = GYRO_COEFFICIENT;
	mpu6050_data.complemantary_filter_settings.ACC_COEFFICIENT = ACC_COEFFICIENT;
	return;
}
