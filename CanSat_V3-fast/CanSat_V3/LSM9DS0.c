/*
 * LSM9DS0.c
 *
 * Created: 2015-05-09 14:55:56
 *  Author: stratus
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include "struct.h"
#include "LSM9DS0.h"
#include "CanSat.h"

void LSM9DS0_Init(void){
	//GYRO
	//--------------Rate 380Hz BW 100Hz-----------------------
	SENSORS_I2C.MASTER.ADDR = 0xD4;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x20;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0xBF;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	//_delay_us(40);
	//------------Scale 2000 deg/s----------------------------
	SENSORS_I2C.MASTER.ADDR = 0xD4;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x23;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x20;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	//_delay_us(40);
	
	//ACCEL
	//-------------Rate 400Hz --------------------------------
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x20;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x87;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	
	//-------------BW 194Hz, +/- 16g--------------------------
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x21;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x60;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	
	//MAG
	//-------------Rate 50Hz Hi res--------------------------
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x24;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0xF0;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	
	//-------------+/-2Gauss--------------------------
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x25;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x00;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	
	//-------------Continuous mode--------------------
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x26;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = 0x00;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
}

void LSM9DS0_Update(LSM9DS0_t * dane){
	
	//GYRO update
	SENSORS_I2C.MASTER.ADDR = 0xD4;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = (0x28 | 0x80);
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = 0xD5;
	dane->raw_gyro_x = I2C_ReadEnd(false);				//0x3B
	dane->raw_gyro_x |= I2C_ReadEnd(false) << 8;		//0x3C
	dane->raw_gyro_y = I2C_ReadEnd(false);				//0x3D
	dane->raw_gyro_y |= I2C_ReadEnd(false) << 8;		//0x3E
	dane->raw_gyro_z = I2C_ReadEnd(false);				//0x3F
	dane->raw_gyro_z |= I2C_ReadEnd(true) << 8;			//0x40
	
	//MAG update
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = (0x08 | 0x80);
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = 0x3B;
	dane->raw_mag_x = I2C_ReadEnd(false);				//0x08
	dane->raw_mag_x |= I2C_ReadEnd(false) << 8;			//0x09
	dane->raw_mag_y = I2C_ReadEnd(false);				//0x0A
	dane->raw_mag_y |= I2C_ReadEnd(false) << 8;			//0x0B
	dane->raw_mag_z = I2C_ReadEnd(false);				//0x0C
	dane->raw_mag_z |= I2C_ReadEnd(true) << 8;			//0x0D
	
	//ACCEL update
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = (0x28 | 0x80);
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = 0x3B;
	dane->raw_accel_x = I2C_ReadEnd(false);				//0x3B
	dane->raw_accel_x |= I2C_ReadEnd(false) << 8;		//0x3C
	dane->raw_accel_y = I2C_ReadEnd(false);				//0x3D
	dane->raw_accel_y |= I2C_ReadEnd(false) << 8;		//0x3E
	dane->raw_accel_z = I2C_ReadEnd(false);				//0x3F
	dane->raw_accel_z |= I2C_ReadEnd(true) << 8;		//0x40
	
	//TEMP update
	SENSORS_I2C.MASTER.ADDR = 0x3A;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = (0x05 | 0x80);
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = 0x3B;
	dane->raw_temp = I2C_ReadEnd(false);			//0x3F
	dane->raw_temp |= I2C_ReadEnd(true) << 8;		//0x40
	
	//calculate values
	dane->accel_x = ((dane->raw_accel_x - dane->offset_accel_x) * 0.732)/1000.0;
	dane->accel_y = ((dane->raw_accel_y - dane->offset_accel_y) * 0.732)/1000.0;
	dane->accel_z = ((dane->raw_accel_z - dane->offset_accel_z) * 0.732)/1000.0;
	dane->gyro_x = ((dane->raw_gyro_x - dane->offset_gyro_x) * 4000.0)/65536.0;
	dane->gyro_y = ((dane->raw_gyro_y - dane->offset_gyro_y) * 4000.0)/65536.0;
	dane->gyro_z = ((dane->raw_gyro_z - dane->offset_gyro_z) * 4000.0)/65536.0;
	dane->mag_x = ((dane->raw_mag_x - dane->offset_mag_x) * 4.0)/655.360;
	dane->mag_y = ((dane->raw_mag_y - dane->offset_mag_y) * 4.0)/655.360;
	dane->mag_z = ((dane->raw_mag_z - dane->offset_mag_z) * 4.0)/655.360;
	dane->temp = 21.0 + dane->raw_temp / 8.0;
}