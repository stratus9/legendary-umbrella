/*
 * Initialization.c
 *
 * Created: 2015-04-26 20:54:17
 *  Author: stratus
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "struct.h"
#include "Initialization.h"
#include "CanSat.h"

void CPU_clk(uint8_t val){
	Osc2MHz((uint8_t)(val/2));
}

void Osc2MHz(uint8_t pll_factor) {
	OSC.CTRL |= OSC_RC2MEN_bm;						// w³¹czenie oscylatora 2MHz
	
	//--------------stabilizacja czêstotliwoœci
	DFLLRC2M.CALA = 0x40;								//dane z sygnatury
	DFLLRC2M.CALB = 0x0D;
	OSC.CTRL|=OSC_RC32KEN_bm;							//w³¹czenie oscylatora 32kHz
	while(!(OSC.STATUS & OSC_RC32KRDY_bm));				//czekanie na ustabilizowanie zegara 32kHz
	OSC.DFLLCTRL &= ~(OSC_RC32MCREF_gm | OSC_RC2MCREF_bm);
	DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
	
	while(!(OSC.STATUS & OSC_RC2MRDY_bm));			// czekanie na ustabilizowanie zegara 2MHz
	CPU_CCP = CCP_IOREG_gc;							// odblokowanie zmiany Ÿród³a sygna³u
	CLK.CTRL = CLK_SCLKSEL_RC2M_gc;					// zmiana Ÿród³a sygna³u na RC 2MHz
	OSC.CTRL &= ~OSC_PLLEN_bm;						// wy³¹czenie PLL
	OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | pll_factor;	// mno¿nik czêstotliwoœci (od 1 do 31)
	OSC.CTRL = OSC_PLLEN_bm;						// w³¹czenie uk³adu PLL
	while(!(OSC.STATUS & OSC_PLLRDY_bm));			// czekanie na ustabilizowanie siê generatora PLL
	CPU_CCP = CCP_IOREG_gc;							// odblokowanie zmiany Ÿród³a sygna³u
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;					// wybór Ÿród³a sygna³u zegarowego PLL
}

void OscRTC(void){
	/*
	OSC.CTRL |= OSC_RC32KEN_bm;						//w³¹czenie oscylatora 32.768kHz
	while(!(OSC.STATUS & OSC_RC32KRDY_bm));			//czekanie na ustabilizowanie siê generatora
	CLK.RTCCTRL = CLK_RTCSRC_RCOSC32_gc;			//RTC zasilane z 32.768kHz
	CLK.RTCCTRL |= CLK_RTCEN_bm;					//w³¹czenie RTC
	*/
}

void RTC_Init(void){
	/*
	RTC.PER = 32768;								//okres 1s
	RTC.CNT = 0;									//zerowanie licznika
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;				//preskaler 1 i w³¹czenie RTC
	RTC.INTCTRL = RTC_OVFINTLVL_MED_gc;				//przerwanie od przepe³nienia - niski priorytet
	*/
}

void TimerCInit(uint16_t period_ms){
	TCC0_CTRLA = TC_CLKSEL_DIV1024_gc;				//w³¹czenie timera z preskalerem 64
	TCC0_CTRLB = 0x00;								//Normal mode
	TCC0_CNT = 0;									//zeruj licznik
	TCC0_PER = (period_ms*32000UL)/1024;			//przeliczenie okresu na 1ms*period
	TCC0_INTCTRLA = TC_OVFINTLVL_MED_gc;			//œredni poziom przerwania od przepe³nienia
}

void TimerDInit(uint16_t period_ms){
	TCD0_CTRLA = TC_CLKSEL_DIV1024_gc;				//w³¹czenie timera z preskalerem 64
	TCD0_CTRLB = 0x00;								//Normal mode
	TCD0_CNT = 0;									//zeruj licznik
	TCD0_PER = (period_ms*32000UL)/1024;			//przeliczenie okresu na 1ms*period
	TCD0_INTCTRLA = TC_OVFINTLVL_LO_gc;				//œredni poziom przerwania od przepe³nienia
}

void TimerEInit(uint16_t period_ms){
	TCE0_CTRLA = TC_CLKSEL_DIV1024_gc;				//w³¹czenie timera z preskalerem 64
	TCE0_CTRLB = 0x00;								//Normal mode
	TCE0_CNT = 0;									//zeruj licznik
	TCE0_PER = (period_ms*32000UL)/1024;			//przeliczenie okresu na 1ms*period
	TCE0_INTCTRLA = TC_OVFINTLVL_MED_gc;			//œredni poziom przerwania od przepe³nienia
}

void TimerFInit(uint16_t period_ms){
	TCF0_CTRLA = TC_CLKSEL_DIV1024_gc;				//w³¹czenie timera z preskalerem 64
	TCF0_CTRLB = 0x00;								//Normal mode
	TCF0_CNT = 0;									//zeruj licznik
	if(period_ms > 2000) period_ms = 2000;			//ograniczenie ze wzglêdu na przepe³nienie
	TCF0_PER = (period_ms*32000UL)/1024;			//przeliczenie okresu na 1ms*period
	TCF0_INTCTRLA = TC_OVFINTLVL_MED_gc;			//œredni poziom przerwania od przepe³nienia
}

void ADC_Init(void){
	ADCA.CTRLA = ADC_ENABLE_bm;
	ADCA.CALL = ReadSignatureByte(0x20) ; //ADC Calibration Byte 0
	ADCA.CALH = ReadSignatureByte(0x21) ; //ADC Calibration Byte 1
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;	//rozdzielczoœæ 12bit, CONVMODE=0
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc;
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
}

void GPS_Init(void){
	GPS_UART_PORT.OUTCLR = PIN2_bm;					//konfiguracja pinów Rx i Tx
	GPS_UART_PORT.OUTSET = PIN3_bm;
	GPS_UART_PORT.DIRSET = PIN3_bm;
	
	GPS_UART.CTRLC = USART_CHSIZE_8BIT_gc;			//ramka: 8bitów, 1 bit stopu, brak bitu parzystoœci
	int16_t BSEL = GPS_BSEL;							//konfiguracja prêdkoœci transmisji 115200
	int8_t BSCALE = GPS_BSCALE;					//-6
	GPS_UART.BAUDCTRLA = (uint8_t)(BSEL&0x00FF);
	GPS_UART.BAUDCTRLB = (uint8_t)(((BSEL >> 8) & 0x000F) | BSCALE) ;
	GPS_UART.CTRLA |= USART_RXCINTLVL_LO_gc;								//odblokowanie przerwañ odbiornika, niski priorytet
	GPS_UART.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;						//w³¹czenie nadajnika i odbiornika USART
}

void GPS_Conf(void){
	/*
	uint8_t i=0;
	char buf[54] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n\0\0";
	while(buf[i]){
		GPS_UART.DATA = buf[i];
		while(!(GPS_UART.STATUS & USART_TXCIF_bm)){
			asm volatile("nop");
		}
		GPS_UART.STATUS |= USART_TXCIF_bm;
		i++;
	}
	*/
}

void USART_Init(void){
	XBEE_UART_PORT.OUTCLR = PIN2_bm;					//konfiguracja pinów Rx i Tx
	XBEE_UART_PORT.OUTSET = PIN3_bm;
	XBEE_UART_PORT.DIRSET = PIN3_bm;
	
	XBEE_UART.CTRLC = USART_CHSIZE_8BIT_gc;	//ramka: 8bitów, 1 bit stopu, brak bitu parzystoœci
	int16_t BSEL = 1047;					//konfiguracja prêdkoœci transmisji 115200
	int8_t BSCALE = 0b10100000;				//-6
	XBEE_UART.BAUDCTRLA = (uint8_t)(BSEL&0x00FF);
	XBEE_UART.BAUDCTRLB = (uint8_t)(((BSEL >> 8) & 0x000F) | BSCALE) ;
	XBEE_UART.CTRLA |= USART_RXCINTLVL_LO_gc | USART_TXCINTLVL_HI_gc;		//odblokowanie przerwañ nadajnika i odbiornika, niski priorytet
	XBEE_UART.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;						//w³¹czenie nadajnika i odbiornika USART
}

void IO_Init(void){
	//konfiguracja przycisków
	PORTE_PIN0CTRL = PORT_OPC_PULLUP_gc;		//w³¹czenie Pull-up przycisku 1
	PORTE_PIN1CTRL = PORT_OPC_PULLUP_gc;		//w³¹czenie Pull-up przycisku 2
	//konfiguracja pinów kontroli GPS
	PORTA_DIR |= PIN0_bm;
	PORTA_OUTCLR = PIN0_bm;
	//konfiguracja LED
	PORTA_DIR |= PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm;	//konfiguracja kierunku pinów LED
	PORTA_OUTCLR = PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm;	//konfiguracja stanu pinów LED
	//konfiguracja buzzera
	PORTF_DIR |= PIN6_bm;						//konfiguracja kierunku pinu buzzera
	PORTF_OUTCLR = PIN6_bm;						//konfiguracja stanu pinu buzzer
	//konfiguracja wyjœæ POWER
	PORTC_DIRSET = PIN2_bm;			//konfiguracja kierunku pinów POWER
	PORTC_OUTCLR = PIN2_bm;			//konfiguracja stanu pinów POWER
	PORTD_DIRSET = PIN1_bm;			//konfiguracja kierunku pinów POWER
	PORTD_OUTCLR = PIN1_bm;			//konfiguracja stanu pinów POWER
	//konfiguracja interfejsu XBEE
	PORTD_OUTCLR = PIN2_bm;		//RX
	PORTD_DIRSET = PIN3_bm;		//TX
	PORTD_DIRSET = PIN5_bm;		//sleep pin
	PORTD_OUTCLR = PIN5_bm;
	//konfiguracja interfejsu GPS
	PORTF_OUTCLR = PIN2_bm;
	PORTF_OUTSET = PIN3_bm;
	PORTF_DIRSET = PIN3_bm;
	//konfiguracja pinów I2C
	PORTC_DIR = PIN0_bm | PIN1_bm;
	PORTC_OUT = PIN0_bm | PIN1_bm;
	//konfiguracja wejœæ data ready
	PORTR_DIRCLR = PIN0_bm;		//LPS
	PORTR_DIRCLR = PIN1_bm;		//LIS
	PORTE_DIRCLR = PIN4_bm;		//MPU
	PORTE_DIRCLR = PIN5_bm;		//LSM
	PORTF_DIRCLR = PIN0_bm;		//LSM
	PORTF_DIRCLR = PIN1_bm;		//HMC
	PORTF_DIRCLR = PIN4_bm;		//LIS2
	PORTD_DIRCLR = PIN4_bm;		//HP
	//konfiguracja wejœæ ADC
	PORTA_OUTCLR = PIN1_bm;		//Vsense2
	PORTA_OUTCLR = PIN7_bm;		//Vsense
	PORTB_OUTCLR = PIN0_bm;		//Vusb
	PORTB_OUTCLR = PIN5_bm;		//light1
	PORTB_OUTCLR = PIN6_bm;		//light2
	PORTB_OUTCLR = PIN7_bm;		//light3
}

void I2C_Init(void){
	SENSORS_I2C.MASTER.CTRLA = TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm;		//ustawienie priorytetu przerwania na Low i w?aczenie przerwania od odbioru
	SENSORS_I2C.MASTER.CTRLB = TWI_MASTER_SMEN_bm;                             //uruchomienie Smart Mode
	SENSORS_I2C.MASTER.BAUD = 15;                                             //BAUD = 48 -> f=300kHz    BAUD = 21 -> f=600kHz
	SENSORS_I2C.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm;                          //w³¹czenie TWI
	SENSORS_I2C.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;                   //I2C wolne
}

void SPI_Init(void){
	SPI_PORT.DIRSET = PIN4_bm;						//podci¹gniêcie CS
	SPI_PORT.DIRSET = PIN5_bm | PIN7_bm;
	SPI_PORT.DIRCLR = PIN6_bm;
	SPI_PORT.OUTSET = PIN5_bm | PIN6_bm | PIN7_bm;	//mo¿e dodaæ PIN6_bm
	//----CS---
	FLASH_CS_PORT.DIRSET = FLASH_CS_PIN;
	FLASH_CS_PORT.OUTSET = FLASH_CS_PIN;
	
	//-----------Clk=250kHz-------------------------
	FLASH_SPI.CTRL = SPI_ENABLE_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV16_gc | SPI_MASTER_bm;
	//FLASH_SPI.INTCTRL = SPI_INTLVL_LO_gc;
}

uint8_t ReadSignatureByte(uint16_t Address)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}