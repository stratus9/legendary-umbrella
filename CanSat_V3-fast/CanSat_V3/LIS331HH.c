/*
 * LIS331HH.c
 *
 * Created: 2015-04-19 10:25:00
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
#include "LIS331HH.h"

void LIS331HH_RegWrite(uint8_t addres, uint8_t data){
	SENSORS_I2C.MASTER.ADDR = 0x32;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = addres;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = data;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(10);
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
}

void LIS331HH_WakeUp(void){
	LIS331HH_RegWrite(0x20,0x2F);	//XYZ enabled, Normal mode, 100Hz ODR
	LIS331HH_RegWrite(0x23,0x30);	//+/-24g
}

void LIS331HH_Update(LIS331HH_t * data){
	SENSORS_I2C.MASTER.ADDR = 0x32;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = (0x28 | 0x80);
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = 0x33;
	int16_t tmp1 = I2C_ReadEnd(false);				//0x3B
	tmp1 |= I2C_ReadEnd(false)<<8;					//0x3C
	int16_t tmp2 = I2C_ReadEnd(false);				//0x3D
	tmp2 |= I2C_ReadEnd(false)<<8;					//0x3E
	int16_t tmp3 = I2C_ReadEnd(false);				//0x3F
	tmp3 |= I2C_ReadEnd(true)<<8;					//
	
	data->raw_accel_x = tmp1;
	data->raw_accel_y = tmp2;
	data->raw_accel_z = tmp3;
}

void LIS331HH_Calc(allData_t * allData){
	float volatile tmp1, tmp2, tmp3;
	float offset = -0.2246;
	
	tmp1 = (allData->LIS331HH->raw_accel_x)*(48/65536.0);
	tmp2 = (allData->LIS331HH->raw_accel_y)*(48/65536.0);
	tmp3 = (allData->LIS331HH->raw_accel_z)*(48/65536.0);
	
	allData->LIS331HH->accel_x = tmp1-offset;
	allData->LIS331HH->accel_y = tmp2-offset;
	allData->LIS331HH->accel_z = tmp3+offset;
}