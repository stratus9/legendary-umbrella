/*
 * MPU9150.c
 *
 * Created: 2015-03-22 19:04:54
 *  Author: stratus
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include "struct.h"
#include "MPU9150.h"
#include "CanSat.h"
#include "I2C.h"

//---------------------------Inicjalizacja czujnika MPU9150---------------------------
void MPU9150_WakeUp(void){
	I2C_WriteReg(0xD0, 0x6B, 0x02);
	I2C_WriteReg(0xD0, 0x1A, 0x02);	//HLPS 94Hz
	I2C_WriteReg(0xD0, 0x1B, 0x18);	//gyro +/-2000 deg/s
	I2C_WriteReg(0xD0, 0x1C, 0x18);	//accel +/-16g
}

//---------------------------Who am I------------------------
uint8_t MPU9150_WhoAmI(void){
	uint8_t dane;
	I2C_ReadRegister(0xD0, 0x75, 1, &dane);
	return dane;
}

//----------------------------Odczyt danych z czujnika---------------------
void MPU9150_RawUpdate(struct MPU9150_t * data){
	uint8_t bufor[20];
	I2C_ReadRegister(0xD0, 0x3B, 14, bufor);
	
	(*data).raw_accel_x = bufor[0] << 8 ;				//0x3B
	(*data).raw_accel_x |= bufor[1];					//0x3C
	(*data).raw_accel_y = bufor[2] << 8;				//0x3D
	(*data).raw_accel_y |= bufor[3];					//0x3E
	(*data).raw_accel_z = bufor[4] << 8;				//0x3F
	(*data).raw_accel_z |= bufor[5];					//0x40
	(*data).raw_temp = bufor[6] << 8;					//0x41
	(*data).raw_temp |= bufor[7];						//0x42
	(*data).raw_gyro_x = bufor[8] << 8;					//0x43
	(*data).raw_gyro_x |= bufor[9];						//0x44
	(*data).raw_gyro_y = bufor[10] << 8;				//0x45
	(*data).raw_gyro_y |= bufor[11];					//0x46
	(*data).raw_gyro_z = bufor[12] << 8;				//0x47
	(*data).raw_gyro_z |= bufor[13];					//0x48
}

//-----------------------------Inicjalizacja magnetometru-------------------------------------------
void MPU9150_MagInit(void){
	I2C_WriteReg(0xD0, 0x6A, 0x00);
	I2C_WriteReg(0xD0, 0x37, 0x02);
}

//---------------------------Odczyt danych z magnetometru------------------------------------------
void MPU9150_MagStartConv(void){
	MPU9150_MagRegWrite(0x0A, 0x01);
}
void MPU9150_MagUpdate(struct MPU9150_t * data){
	uint16_t tmp1, tmp2, tmp3;
	
	SENSORS_I2C.MASTER.ADDR = 0x18;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x03;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = 0x19;
	tmp1 = I2C_ReadEnd(false);			//0x03
	tmp1 |= I2C_ReadEnd(false) << 8;		//0x04
	tmp2 = I2C_ReadEnd(false);			//0x05
	tmp2 |= I2C_ReadEnd(false) << 8;		//0x06
	tmp3 = I2C_ReadEnd(false);			//0x07
	tmp3 |= I2C_ReadEnd(true) << 8;		//0x08

	(*data).raw_mag_x = tmp1;
	(*data).raw_mag_y = tmp2;
	(*data).raw_mag_z = tmp3;
}

//------------------------Konwersja pomaiarów magnetometru na jednostki inŸ----------------------
void MPU9150_MagCalc(struct MPU9150_t * data){
	(*data).mag_x = ((*data).raw_mag_x + (((*data).raw_mag_x * (*data).sens_mag_x) >> 8)) * 3;
	(*data).mag_y = ((*data).raw_mag_y + (((*data).raw_mag_y * (*data).sens_mag_y) >> 8)) * 3;
	(*data).mag_z = ((*data).raw_mag_z + (((*data).raw_mag_z * (*data).sens_mag_z) >> 8)) * 3;
}

//-----------------------Kalibracja magnetometru-------------------------------------------------
void MPU9150_MagCal(struct MPU9150_t * data){
	MPU9150_MagRegWrite(0x0A, 0x00);		//powerdown mode
	_delay_us(100);
	MPU9150_MagRegWrite(0x0A, 0xFF);		//Fuse ROM mode
	_delay_us(100);
	
	
	uint8_t x, y, z;						//read sensitivity adjustment data for the 3 axes
	SENSORS_I2C.MASTER.ADDR = 0x18;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x10;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = 0x19;
	x = I2C_ReadEnd(false);				//0x03
	y = I2C_ReadEnd(false);				//0x04
	z = I2C_ReadEnd(true);				//0x05
	
	MPU9150_MagRegWrite(0x0A, 0x00);		//powerdown mode
	
	
	x = (x-128);
	y = (y-128);
	z = (z-128);
	
	(*data).sens_mag_x = x;
	(*data).sens_mag_y = y;
	(*data).sens_mag_z = z;
}

void MPU9150_Conv(struct MPU9150_t * MPU9150,struct frame_t * frame){
	(*MPU9150).mag_x = ((*MPU9150).raw_mag_x + (((*MPU9150).raw_mag_x * (*MPU9150).sens_mag_x) >> 8)) * 0.3;
	(*MPU9150).mag_y = ((*MPU9150).raw_mag_y + (((*MPU9150).raw_mag_y * (*MPU9150).sens_mag_y) >> 8)) * 0.3;
	(*MPU9150).mag_z = ((*MPU9150).raw_mag_z + (((*MPU9150).raw_mag_z * (*MPU9150).sens_mag_z) >> 8)) * 0.3;
	
	(*MPU9150).accel_x = ((*MPU9150).raw_accel_x - (*MPU9150).offset_accel_x) / 2048.0;	//16384.0; -2g
	(*MPU9150).accel_y = ((*MPU9150).raw_accel_y - (*MPU9150).offset_accel_y) / 2048.0; //16384.0;
	(*MPU9150).accel_z = ((*MPU9150).raw_accel_z - (*MPU9150).offset_accel_z) / 2048.0; //16384.0;
	
	(*MPU9150).gyro_x = ((*MPU9150).raw_gyro_x - (*MPU9150).offset_gyro_x) / 16.4;	//131 - 250deg/s, 62.5 - 500deg/s,32.8 - 1000deg/s,16.4 - 2000deg/s
	(*MPU9150).gyro_y = ((*MPU9150).raw_gyro_y - (*MPU9150).offset_gyro_y) / 16.4;
	(*MPU9150).gyro_z = ((*MPU9150).raw_gyro_z - (*MPU9150).offset_gyro_z) / 16.4;
	
	(*MPU9150).temp  = ((*MPU9150).raw_temp + 11900.0) / 34.0;	//chyba *10
	
	//------------store to frame struct-------------
	frame->MPU9150_temp = (MPU9150->temp)/10.0;
	frame->MPU9150_gyro_x = MPU9150->gyro_x;
	frame->MPU9150_gyro_y = MPU9150->gyro_y;
	frame->MPU9150_gyro_z = MPU9150->gyro_z;
	frame->MPU9150_accel_x = MPU9150->accel_x;
	frame->MPU9150_accel_y = MPU9150->accel_y;
	frame->MPU9150_accel_z = MPU9150->accel_z;
	frame->MPU9150_mag_x = MPU9150->mag_x;
	frame->MPU9150_mag_y = MPU9150->mag_y;
	frame->MPU9150_mag_z = MPU9150->mag_z;	
}