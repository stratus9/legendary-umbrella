/*
 * LPS25H.c
 *
 * Created: 2015-04-18 09:27:46
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
#include "I2C.h"

void LPS25H_config(void){
	//--------------uœrednianie: press 512, temp 64-----------
	I2C_WriteReg(0xB8,0x10,0x0F);
	//------------Data ready interrupt------------------------
	I2C_WriteReg(0xB8,0x23,0x01);
	//-------------active mode, 25Hz output-------------------
	I2C_WriteReg(0xB8,0x20,0xC0);
}

void LPS25H_update(struct LPS25H_t * LPS25H){
	uint8_t bufor123[10];
	I2C_ReadRegister(0xB8, (0x28 | 0x80), 5, bufor123);
	
	LPS25H->raw_pressure = (uint32_t)(bufor123[0]) | (uint32_t)(bufor123[1])<<8 | (uint32_t)(bufor123[2])<<16;
	LPS25H->raw_temp = bufor123[3] | (bufor123[4]<<8);
}

void LPS25H_calc(struct LPS25H_t * LPS25H, struct frame_t * frame){
	float x1;
	x1 = LPS25H->raw_pressure;
	LPS25H->pressure = x1/4096.0;
	x1 = LPS25H->raw_temp;
	LPS25H->temp = x1/480.0+42.5;
	
	//check data
	
	//save to frame struct
	frame->LPS25H_pressure = LPS25H->pressure;
	frame->LPS25H_temp = LPS25H->temp;
}

uint8_t LPS25H_WhoIAm(void){
	uint8_t tmp;
	I2C_ReadRegister(0xB8,0x0F,1,&tmp);
	return tmp;
}