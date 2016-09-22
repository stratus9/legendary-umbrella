/*
 * Initialization.h
 *
 * Created: 2015-04-26 20:54:25
 *  Author: stratus
 */ 


#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

void CPU_clk(uint8_t);
void Osc2MHz(uint8_t);
void OscRTC(void);
void RTC_Init(void);
void TimerCInit(uint16_t);
void TimerDInit(uint16_t);
void TimerEInit(uint16_t);
void TimerFInit(uint16_t);
void ADC_Init(void);
void GPS_Init(void);
void USART_Init(void);
void IO_Init(void);
void I2C_Init(void);
void SPI_Init(void);
uint8_t ReadSignatureByte(uint16_t);
void GPS_Conf(void);
#endif /* INITIALIZATION_H_ */
