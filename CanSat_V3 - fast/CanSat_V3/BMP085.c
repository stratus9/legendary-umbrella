/*
 * BMP085.c
 *
 * Created: 2015-04-17 16:44:28
 *  Author: stratus
 */ 
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "struct.h"
#include "CanSat.h"

void BMP085_CalUpdate(struct BMP085_t * BMP085){
	/*
	TWIC_MASTER_ADDR = 0xEE;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC_MASTER_DATA = 0xAA;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	_delay_us(40);
	
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	TWIC_MASTER_ADDR = 0xEF;
	BMP085->ac1 = I2C_ReadEnd(false) << 8 ;
	BMP085->ac1 |= I2C_ReadEnd(false);
	BMP085->ac2 = I2C_ReadEnd(false) << 8;
	BMP085->ac2 |= I2C_ReadEnd(false);
	BMP085->ac3 = I2C_ReadEnd(false) << 8;
	BMP085->ac3 |= I2C_ReadEnd(false);
	BMP085->ac4 = I2C_ReadEnd(false) << 8;
	BMP085->ac4 |= I2C_ReadEnd(false);
	BMP085->ac5 = I2C_ReadEnd(false) << 8;
	BMP085->ac5 |= I2C_ReadEnd(false);
	BMP085->ac6 = I2C_ReadEnd(false) << 8;
	BMP085->ac6 |= I2C_ReadEnd(false);
	BMP085->b1 = I2C_ReadEnd(false) << 8;
	BMP085->b1 |= I2C_ReadEnd(false);
	BMP085->b2 = I2C_ReadEnd(false) << 8;
	BMP085->b2 |= I2C_ReadEnd(false);
	BMP085->mb = I2C_ReadEnd(false) << 8;
	BMP085->mb |= I2C_ReadEnd(false);
	BMP085->mc = I2C_ReadEnd(false) << 8;
	BMP085->mc |= I2C_ReadEnd(false);
	BMP085->md = I2C_ReadEnd(false) << 8;
	BMP085->md |= I2C_ReadEnd(true);
	*/
}

void BMP085_TempUpdate(struct BMP085_t * BMP085){
	uint16_t tmp;
	TWIC_MASTER_ADDR = 0xEE;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC_MASTER_DATA = 0xF6;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	TWIC_MASTER_ADDR = 0xEF;
	tmp = I2C_ReadEnd(false) << 8 ;
	tmp |= I2C_ReadEnd(true);
	BMP085->ut = tmp;
}

void BMP085_PressUpdate(struct BMP085_t * BMP085){
	uint16_t tmp;
	TWIC_MASTER_ADDR = 0xEE;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC_MASTER_DATA = 0xF6;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	//_delay_us(40);
	
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	TWIC_MASTER_ADDR = 0xEF;
	tmp = I2C_ReadEnd(false) << 8 ;
	tmp |= I2C_ReadEnd(true);
	BMP085->up = tmp;
}

void BMP085_StartTempConv(void){
	TWIC_MASTER_ADDR = 0xEE;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC_MASTER_DATA = 0xF4;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC_MASTER_DATA = 0x2E;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	//_delay_us(40);
}

void BMP085_StartPressConv(void){
	TWIC_MASTER_ADDR = 0xEE;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC_MASTER_DATA = 0xF4;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC_MASTER_DATA = 0xF4;
	while(!(TWIC_MASTER_STATUS & TWI_MASTER_WIF_bm)) {}
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	//_delay_us(40);
}

void BMP085_Conv(struct BMP085_t * BMP085, struct frame_t * frame){
	int32_t X1,X2,B5;
	uint32_t B4, B7;
	int32_t B6, X3, B3, p;
	int32_t UT, UP;
	uint8_t oss = 0;
	int16_t ac1 = BMP085->ac1;
	int16_t ac2 = BMP085->ac2;
	int16_t ac3 = BMP085->ac3;
	uint16_t ac4 = BMP085->ac4;
	uint16_t ac5 = BMP085->ac5;
	uint16_t ac6 = BMP085->ac6;
	int16_t b1 = BMP085->b1;
	int16_t b2 = BMP085->b2;
	int16_t mb = BMP085->mb;
	int16_t mc = BMP085->mc;
	short md = BMP085->md;
	UT = BMP085->ut;
	UP = BMP085->up;
	//------------Temperature compensation
	X1 = ((int32_t)UT - (int32_t)(ac6));
	X1 = (X1*(int32_t)(ac5))>>10;
	X1 = X1 >> 5;
	X2 = (X1+(int32_t)(md));
	X2 = ((int32_t)(mc)<<11)/X2;
	B5 = X1+X2;
	BMP085->temperature = ((B5+8)>>4)/10.0;
	//------------Pressure compensation
	B6 = B5-4000;
	X1 = (b2*((B6*B6)>>12))>>11;
	X2 = (ac2*B6)>>11;
	X3 = X1+X2;
	B3 = (((int32_t)(ac1))*4)+X3;
	B3 = ((B3<<oss)+2)>>2;
	X1 = (ac3*B6)>>13;
	X2 = (b1*((B6*B6)>>12))>>16;
	X3 = ((X1+X2)+2)>>2;
	B4 = ac4*(uint32_t)(X3+32768)>>15;
	B7 = ((UP-B3));
	B7 = B7*(50000>>oss);
	p = B7 < 0x80000000 ? (B7 * 2) / B4 : (B7 / B4) * 2;
	X1 = (p>>8)*(p>>8);
	X1 = (X1*3038)>>16;
	X2 = (-7357*p)>>16;
	p = p+((X1+X2+3791)>>4);
	BMP085->pressure = p/100.0;
	
	//------------------Save to frame struct--------------
	frame->BMP085_pressure = (BMP085->pressure);
	frame->BMP085_temp = (BMP085->temperature);
}

void BMP085_FastCal(struct BMP085_t * BMP085){
	BMP085->ac1 = 7115;
	BMP085->ac2 = -1247;
	BMP085->ac3 = -14436;
	BMP085->ac4 = 33753;
	BMP085->ac5 = 25479;
	BMP085->ac6 = 17226;
	BMP085->b1 = 5498;
	BMP085->b2 = 68;
	BMP085->mb = -32768;
	BMP085->mc = -11075;
	BMP085->md = 2432;
}