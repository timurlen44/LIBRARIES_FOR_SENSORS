#include "JN2101_LIB.h"
#include "strings.h"
extern CAN_HandleTypeDef hcan;
void init_jn2101_lib()
{
	for(int i = 0;i<8;i++)
	{
		send_data_to_jn2101 [i]      =  0x00;
		received_data_from_jn2101[i] =  0x00;
	}
	is_jn2101_connected();

	return;
}
void read_jn2101_node_id()
{
	send_data_to_jn2101 [0] =  0x40;//Command Byte
	send_data_to_jn2101 [1] =  0x00;//LSB of Index
	send_data_to_jn2101 [2] =  0x20;//MSB of Index
	send_data_to_jn2101 [3] =  0x00;//Sub Index
	send_data_to_jn2101 [4] =  0x00;
	send_data_to_jn2101 [5] =  0x00;
	send_data_to_jn2101 [6] =  0x00;
	send_data_to_jn2101 [7] =  0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);


	return;
}

void read_jn2101_vendor_id()
{
	send_data_to_jn2101 [0] =  0x40;//Command Byte
	send_data_to_jn2101 [1] =  0x18;//LSB of Index
	send_data_to_jn2101 [2] =  0x10;//MSB of Index
	send_data_to_jn2101 [3] =  0x01;//Sub Index
	send_data_to_jn2101 [4] =  0x00;
	send_data_to_jn2101 [5] =  0x00;
	send_data_to_jn2101 [6] =  0x00;
	send_data_to_jn2101 [7] =  0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
	return;
}

void read_jn2101_product_code()
{
	send_data_to_jn2101 [0] =  0x40;//Command Byte
	send_data_to_jn2101 [1] =  0x18;//LSB of Index
	send_data_to_jn2101 [2] =  0x10;//MSB of Index
	send_data_to_jn2101 [3] =  0x02;//Sub Index
	send_data_to_jn2101 [4] =  0x00;
	send_data_to_jn2101 [5] =  0x00;
	send_data_to_jn2101 [6] =  0x00;
	send_data_to_jn2101 [7] =  0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
	return;
}

void read_jn2101_revision_number()
{
	send_data_to_jn2101 [0] =  0x40;//Command Byte
	send_data_to_jn2101 [1] =  0x18;//LSB of Index
	send_data_to_jn2101 [2] =  0x10;//MSB of Index
	send_data_to_jn2101 [3] =  0x03;//Sub Index
	send_data_to_jn2101 [4] =  0x00;
	send_data_to_jn2101 [5] =  0x00;
	send_data_to_jn2101 [6] =  0x00;
	send_data_to_jn2101 [7] =  0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
	return;
}
void read_jn2101_serial_number()
{
	send_data_to_jn2101 [0] =  0x40;//Command Byte
	send_data_to_jn2101 [1] =  0x18;//LSB of Index
	send_data_to_jn2101 [2] =  0x10;//MSB of Index
	send_data_to_jn2101 [3] =  0x04;//Sub Index
	send_data_to_jn2101 [4] =  0x00;
	send_data_to_jn2101 [5] =  0x00;
	send_data_to_jn2101 [6] =  0x00;
	send_data_to_jn2101 [7] =  0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
	return;
}

void read_jn2101_canbus_baudrate()
{
	send_data_to_jn2101 [0] =  0x40;//Command Byte
	send_data_to_jn2101 [1] =  0x01;//LSB of Index
	send_data_to_jn2101 [2] =  0x20;//MSB of Index
	send_data_to_jn2101 [3] =  0x00;//Sub Index
	send_data_to_jn2101 [4] =  0x00;
	send_data_to_jn2101 [5] =  0x00;
	send_data_to_jn2101 [6] =  0x00;
	send_data_to_jn2101 [7] =  0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);

	return;
}


void read_jn2101_FIR_filter_hz()
{
	send_data_to_jn2101[0] = 0x40; //Command Byte
	send_data_to_jn2101[1] = 0x43; //LSB of Index
	send_data_to_jn2101[2] = 0x20; //MSB of Index
	send_data_to_jn2101[3] = 0x00; //Sub Index
	send_data_to_jn2101[4] = 0x00;
	send_data_to_jn2101[5] = 0x00;
	send_data_to_jn2101[6] = 0x00;
	send_data_to_jn2101[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
	//jn2101_FIR_Filter_Hz
	return;
}

void set_jn2101_FIR_filter_hz(enum FIR_Filter_Hz_Enum FIR_Filter_Hz)
{
	send_data_to_jn2101[0] = 0x22; //Command Byte
	send_data_to_jn2101[1] = 0x43; //LSB of Index
	send_data_to_jn2101[2] = 0x20; //MSB of Index
	send_data_to_jn2101[3] = 0x00; //Sub Index
	send_data_to_jn2101[5] = 0x00;
	send_data_to_jn2101[6] = 0x00;
	send_data_to_jn2101[7] = 0x00;
	if(FIR_Filter_Hz == _FIR_Deactivated)
	{
		send_data_to_jn2101[4] = 0x00;
	}
	else if(FIR_Filter_Hz == _FIR_Ten_Hz)
	{
		send_data_to_jn2101[4] = 0x01;
	}
	else if(FIR_Filter_Hz == _FIR_Five_Hz)
	{
		send_data_to_jn2101[4] = 0x02;
	}
	else if(FIR_Filter_Hz == _FIR_One_Hz)
	{
		send_data_to_jn2101[4] = 0x03;
	}
	else if(FIR_Filter_Hz == _FIR_Half_Hz)
	{
		send_data_to_jn2101[4] = 0x04;
	}
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
	//jn2101_FIR_Filter_Hz
	return;
}

void is_jn2101_connected()
{
	send_data_to_jn2101[0] = 0x40;  //Command Byte
	send_data_to_jn2101[1] = 0x18;  //LSB of Index
	send_data_to_jn2101[2] = 0x10;  //MSB of Index
	send_data_to_jn2101[3] = 0x00;  //Sub Index
	send_data_to_jn2101[4] = 0x00;
	send_data_to_jn2101[5] = 0x00;
	send_data_to_jn2101[6] = 0x00;
	send_data_to_jn2101[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
	return;
}

void read_jn2101_x_axis()
{
	send_data_to_jn2101[0] = 0x40;  //Command Byte
	send_data_to_jn2101[1] = 0x10;  //LSB of Index
	send_data_to_jn2101[2] = 0x60;  //MSB of Index
	send_data_to_jn2101[3] = 0x00;  //Sub Index
	send_data_to_jn2101[4] = 0x00;
	send_data_to_jn2101[5] = 0x00;
	send_data_to_jn2101[6] = 0x00;
	send_data_to_jn2101[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
}
void read_jn2101_y_axis()
{
	send_data_to_jn2101[0] = 0x40;  //Command Byte
	send_data_to_jn2101[1] = 0x20;  //LSB of Index
	send_data_to_jn2101[2] = 0x60;  //MSB of Index
	send_data_to_jn2101[3] = 0x00;  //Sub Index
	send_data_to_jn2101[4] = 0x00;
	send_data_to_jn2101[5] = 0x00;
	send_data_to_jn2101[6] = 0x00;
	send_data_to_jn2101[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan, &TxTransmitter, send_data_to_jn2101, &MailBox);
}
