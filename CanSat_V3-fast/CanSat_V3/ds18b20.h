/*
   Plik ds18b20.h

   (xyz.isgreat.org)  
*/

#ifndef DS18B20_H
#define DS18B20_H

/* DS18B20 przy³¹czony do portu  	PD7 AVRa  */
#define SET_ONEWIRE_PORT     	PORTE_OUTSET  = PIN2_bm
#define CLR_ONEWIRE_PORT     	PORTE_OUTCLR  = PIN2_bm
#define IS_SET_ONEWIRE_PIN   	PORTE_IN & PIN2_bm
#define SET_OUT_ONEWIRE_DDR  	PORTE_DIRSET  = PIN2_bm
#define SET_IN_ONEWIRE_DDR   	PORTE_DIRCLR  = PIN2_bm

unsigned char ds18b20_ConvertT(void);
int ds18b20_Read(unsigned char []);
int ds18b20_ReadROM(unsigned char []);
void OneWireStrong(char);
unsigned char OneWireReset(void);
void OneWireWriteByte(unsigned char);
unsigned char OneWireReadByte(void);

#endif