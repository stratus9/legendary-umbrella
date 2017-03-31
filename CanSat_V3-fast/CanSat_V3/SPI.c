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
#include "CanSat.h"

void SPI_W_Byte(uint8_t byte){
	FLASH_SPI.DATA = byte;
	while(!(FLASH_SPI.STATUS & SPI_IF_bm)){}
	volatile char tmp = FLASH_SPI.DATA;
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
	char tmp1, tmp2, tmp3;
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
	SPI_WriteEnable();
	SPI_CS(true);
	SPI_W_Byte(0x60);	//Chip erase
	SPI_CS(false);
	_delay_us(100);
	
	SPI_WriteFin();
	SPI_WriteDisable();
}

char SPI_Read(uint32_t address,uint16_t size, char * tablica){
	char tmp=0;
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

char SPI_ReadByte(uint32_t address){
	char tmp;
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

void SPI_WriteProtection(bool block){
	if(block){
		SPI_CS(true);
		SPI_W_Byte(0x50);	//Enable-Write-StatusRegister
		SPI_CS(false);
		_delay_us(1);
		SPI_CS(true);
		SPI_W_Byte(0x01);	//Write-Status-Register
		SPI_W_Byte(0x1C);	//Protect memory
		SPI_CS(false);
	}
	else{
		SPI_CS(true);
		SPI_W_Byte(0x50);	//Enable-Write-StatusRegister
		SPI_CS(false);
		_delay_us(1);
		SPI_CS(true);
		SPI_W_Byte(0x01);	//Write-Status-Register
		SPI_W_Byte(0x00);	//Unprotect memory
		SPI_CS(false);
	}
}

void SPI_CmdSend(char cmd){
	SPI_CS(true);		//rozpoczêcie transmisji
	SPI_W_Byte(cmd);	//hardware End-of_Write
	SPI_CS(false);		//zakoñczenie transmisji
}

void SPI_AAI_Mode_Start(void){
	SPI_WriteEnable();
	_delay_us(1);
	SPI_CmdSend(0x70);	//Hardware End-of_Write
	_delay_us(1);
}

void SPI_AAI_Mode_Stop(void){
	SPI_WriteDisable();
	_delay_us(1);
	SPI_CmdSend(0x80);	//Hardware End-of_Write disable
	_delay_us(1);
}

/*  Coœ do poprawy!!!!!!!!!!!!!!!!!!1
void SPI_PagueWrite(uint16_t page, uint16_t page_length, frame_t * frame){
	uint16_t i = 0;
	SPI_AAI_Mode_Start();				//konfiguracja transmisji AAI
	
	SPI_CS(true);						//rozpoczêcie transmisji
	SPI_W_Byte(0x02);					//polecenie zapisu strony
	uint32_t adres = page*page_length;	//calculate start address
	SPI_W_Byte((adres>>16) & 0xFF);		//address MSB
	SPI_W_Byte((adres>>8) & 0xFF);		//address cd.
	SPI_W_Byte(adres & 0xFF);			//address LSB
	//data start
	while(i<page_length){
		SPI_W_Byte(frame->frameASCII[i++]);
		SPI_W_Byte(frame->frameASCII[i++]);
		while(PORTC.IN & PIN6_bm){}
	}
	SPI_CS(false);						//zakoñczenie transmisji
	_delay_us(1);
	SPI_AAI_Mode_Stop();				//exit AAI mode
}*/

void SPI_WriteFin(void){
	SPI_CS(true);
	SPI_W_Byte(0x05);	//status register
	volatile char ch = SPI_R_Byte();
	do{
		ch = SPI_R_Byte();
	}
	while(ch & 0x01);
	SPI_CS(false);
}

/*
void SPI_PageWrite(uint16_t page, uint16_t page_length, frame_t * frame){
	uint16_t i = 0;
	while(i < page_length){
		uint32_t adres = page*page_length+i;	//calculate start address
		SPI_WriteByte(adres,frame->frameASCII[i++]);
		SPI_WriteFin();
		PORTA_OUTTGL = PIN2_bm;
	}
}
*/

uint32_t SPI_FindEnd(void){
	SPI_CS(true);
	SPI_W_Byte(0x03);			//Read
	SPI_W_Byte(0);				//address MSB
	SPI_W_Byte(0);				//address cd.
	SPI_W_Byte(0);				//address LSB
	uint32_t n=0;
	while((n < 4100000) && (SPI_R_Byte() != 0xFF)) n++;	//szukaj pocz¹tku wolnej pamiêci (0xFF)
	SPI_CS(false);
	return n;
}

void SPI_WriteFrame(uint32_t * adres, uint16_t frame_length, frame_t * frame){
	uint32_t i = 0;
	if((*adres) < 4194000){
		PORTA_OUTSET = PIN2_bm;
		while((frame->frameASCII[i]) && (i<frame_length)){
			SPI_WriteByte((*adres),frame->frameASCII[i++]);
			SPI_WriteFin();
			(*adres)++;			//address increment
		}
		PORTA_OUTCLR = PIN2_bm;
	}
}