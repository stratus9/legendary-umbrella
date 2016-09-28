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
#include "util.h"

void LPS25H_config(void){
	//--------------uœrednianie: press 512, temp 64-----------
	I2C_WriteReg(0xB8,0x10,0x0F);
	//------------Data ready interrupt------------------------
	I2C_WriteReg(0xB8,0x23,0x01);
	//-------------active mode, 25Hz output-------------------
	I2C_WriteReg(0xB8,0x20,0xC0);
}

void LPS25H_update(LPS25H_t * LPS25H){
	uint8_t bufor123[10];
	I2C_ReadRegister(0xB8, (0x28 | 0x80), 5, bufor123);
	
	LPS25H->raw_pressure = (uint32_t)(bufor123[0]) | (uint32_t)(bufor123[1])<<8 | (uint32_t)(bufor123[2])<<16;
	LPS25H->raw_temp = bufor123[3] | (bufor123[4]<<8);
}

void LPS25H_calc(LPS25H_t * LPS25H){
	float x1;
	x1 = LPS25H->raw_pressure;
	LPS25H->pressure = x1/4096.0;
	x1 = LPS25H->raw_temp;
	LPS25H->temp = x1/480.0+42.5;
	
	//check data
}


void altitudeCalcLPS(LPS25H_t * LPS25H){
	//----Calculate new altitude----------------------------
	if(LPS25H->start_pressure == 0) LPS25H->start_pressure = LPS25H->pressure;
	float new_altitude = altitudeCalc(LPS25H->pressure, LPS25H->start_pressure);
	
	//----Exponenetal filtering-----------------------------
	float old_altitude = LPS25H->altitude;
	LPS25H->altitude = old_altitude*(1-LPS25H_alti_alpha) + new_altitude*LPS25H_alti_alpha;	//Exponential smoothing
	
	//----Ascent velocity-----------------------------------
	LPS25H->velocity = LPS25H->velocity*(1-LPS25H_velo_alpha) + (LPS25H->altitude-old_altitude)*LPS25H_velo_alpha* sampling_rate;
}

uint8_t LPS25H_WhoIAm(void){
	uint8_t tmp;
	I2C_ReadRegister(0xB8,0x0F,1,&tmp);
	return tmp;
}