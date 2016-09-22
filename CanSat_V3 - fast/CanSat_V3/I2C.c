/*
 * I2C.c
 *
 * Created: 2015-10-25 10:44:04
 *  Author: baretk
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "struct.h"
#include "I2C.h"
#include "CanSat.h"

uint8_t I2C_ReadEnd(bool koniec){
	uint8_t dane;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_RIF_bm)) {}
	if(koniec) SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
	dane = SENSORS_I2C.MASTER.DATA;
	return dane;
}

void I2C_WriteReg(uint8_t I2Caddres, uint8_t reg,  uint8_t  data){
	SENSORS_I2C.MASTER.ADDR = I2Caddres;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = reg;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = data;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
}

void I2C_ReadRegister(uint8_t I2Caddres, uint8_t reg, uint8_t length, uint8_t * data){
	SENSORS_I2C.MASTER.ADDR = I2Caddres;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}
	SENSORS_I2C.MASTER.DATA = reg;
	while(!(SENSORS_I2C.MASTER.STATUS & TWI_MASTER_WIF_bm)) {}

	SENSORS_I2C.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	SENSORS_I2C.MASTER.ADDR = I2Caddres | 0x01;
	for(uint8_t i=0; i<length-1; i++){
		*data = I2C_ReadEnd(false);
		data++;
	}
	*data = I2C_ReadEnd(true);
}