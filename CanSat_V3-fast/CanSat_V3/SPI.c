/*
 * SPI.c
 *
 * Created: 2015-05-15 11:45:16
 *  Author: stratus
 */ 
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "struct.h"
#include "SPI.h"
#include "util.h"
#include "CanSat.h"

void SPI_W_Byte(uint8_t byte){
	FLASH_SPI.DATA = byte;
	while(!(FLASH_SPI.STATUS & SPI_IF_bm)){}
	volatile uint8_t tmp = FLASH_SPI.DATA;
}

uint8_t SPI_R_Byte(void){
	FLASH_SPI.DATA = 0x00;
	while(!(FLASH_SPI.STATUS & SPI_IF_bm)){}
	return FLASH_SPI.DATA;
}

void SPI_CS(bool enable){
	if(enable) FLASH_CS_PORT.OUTCLR = FLASH_CS_PIN;
	else FLASH_CS_PORT.OUTSET = FLASH_CS_PIN;
}

bool SPI_MemoryCheck(void){
	uint8_t tmp1, tmp2, tmp3;
	SPI_CS(true);
	SPI_W_Byte(0x9F);
	tmp1 = SPI_R_Byte();	//BF
	tmp2 = SPI_R_Byte();	//Device Type
	tmp3 = SPI_R_Byte();	//Memory Cap
	SPI_CS(false);
	if((tmp1 == 0xbf) && (tmp2 == 0x25) && (tmp3 == 0x4A)) return true;
	else return false;
}

void SPI_WriteEnable(void){
	SPI_CS(true);
	SPI_W_Byte(0x06);
	SPI_CS(false);
}

void SPI_WriteDisable(void){
	SPI_CS(true);
	SPI_W_Byte(0x04);
	SPI_CS(false);
}

uint8_t SPI_Status(void){
	SPI_CS(true);
	SPI_W_Byte(0x05);	//status register
	uint8_t tmp = SPI_R_Byte();
	SPI_CS(false);
	return tmp;
}

void SPI_ChipErase(void){
	SPI_WriteReady();
	SPI_WriteEnable();
	SPI_CS(true);
	SPI_W_Byte(0xC7);	//Chip erase
	SPI_CS(false);
	_delay_us(100);
	
	SPI_WriteReady();
	SPI_WriteDisable();
}

uint8_t SPI_Read(uint32_t address,uint16_t size, uint8_t * tablica){
	uint8_t tmp=0;
	SPI_CS(true);
	SPI_W_Byte(0x03);					//Read
	SPI_W_Byte((address>>16) & 0xFF);	//address MSB
	SPI_W_Byte((address>>8) & 0xFF);	//address cd.
	SPI_W_Byte(address & 0xFF);			//address LSB
	uint16_t i=0;
	for(i=0;i<size;i++){
		tmp = SPI_R_Byte();
		*(tablica++) = tmp;
	}
	SPI_CS(false);
	return tmp;
}

uint8_t SPI_ReadByte(uint32_t address){
	uint8_t tmp;
	SPI_CS(true);
	SPI_W_Byte(0x03);					//Read
	SPI_W_Byte((address>>16) & 0xFF);	//address MSB
	SPI_W_Byte((address>>8) & 0xFF);	//address cd.
	SPI_W_Byte(address & 0xFF);			//address LSB
	tmp = SPI_R_Byte();
	SPI_CS(false);
	return tmp;
}

void SPI_WriteByte(uint32_t address, uint8_t data){
	SPI_WriteEnable();
	SPI_CS(true);
	SPI_W_Byte(0x02);					//Write
	SPI_W_Byte((address>>16) & 0xFF);	//address MSB
	SPI_W_Byte((address>>8) & 0xFF);	//address cd.
	SPI_W_Byte(address & 0xFF);			//address LSB
	SPI_W_Byte(data);					//dane do zapisu
	SPI_CS(false);
	//SPI_WriteDisable();
}

<<<<<<< HEAD
void SPI_WriteNBytes(uint32_t address, uint8_t * data, uint8_t len){
=======
void SPI_WriteNBytes(uint32_t address, uint8_t * data, uint16_t len){
>>>>>>> origin/master
	SPI_WriteReady();
	SPI_WriteEnable();
	
	SPI_CS(true);
	SPI_W_Byte(0x02);					//Write
	SPI_W_Byte((address>>16) & 0xFF);	//address MSB
	SPI_W_Byte((address>>8) & 0xFF);	//address cd.
	SPI_W_Byte( address & 0xFF);		//address LSB
<<<<<<< HEAD
=======
	
	//SPI_Flash_DMA_transfer_blocking_start(data, len);
>>>>>>> origin/master
	uint8_t i = 0;
	while(i<len){
		SPI_W_Byte(*data++);
		i++;
	}
	SPI_CS(false);
}

void SPI_CmdSend(uint8_t cmd){
	SPI_CS(true);		//rozpoczêcie transmisji
	SPI_W_Byte(cmd);	//hardware End-of_Write
	SPI_CS(false);		//zakoñczenie transmisji
}

void SPI_WriteReady(void)
{
	SPI_CS(true);
	SPI_W_Byte(0x05);	//status register
	volatile uint8_t ch = SPI_R_Byte();
	do{
		ch = SPI_R_Byte();
	}while(ch & 0x01);
	SPI_CS(false);
}

uint32_t SPI_FindEnd(uint32_t length){
	SPI_CS(true);
	SPI_W_Byte(0x03);			//Read
	SPI_W_Byte(0);				//address MSB
	SPI_W_Byte(0);				//address cd.
	SPI_W_Byte(0);				//address LSB
	uint32_t n=0;
	while((n < 4100000) && (SPI_R_Byte() != 0xFF)) n++;	//szukaj pocz¹tku wolnej pamiêci (0xFF)
	SPI_CS(false);
	if(n%length) n = n+length-n%length;
	return n;
}


void SPI_WriteFrame(uint32_t * adres, uint16_t frame_length, uint8_t * frame){
	if((*adres) < 4194000){
		SPI_WriteReady();	//sprawdzenie czy pamiêæ gotowa
		SPI_WriteNBytes((*adres), frame, frame_length);	//d³ugoœæ musi byæ wielokrotnoœci¹ 2
		*adres += frame_length;
		PORTA_OUTTGL = PIN2_bm;
	}
}

void SPI_WriteProtection() {
	SPI_WriteEnable();
	SPI_CmdSend(0x98);	//odblokuj wszystko
}