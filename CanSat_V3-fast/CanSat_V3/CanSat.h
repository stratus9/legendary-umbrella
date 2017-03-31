/*
 * CanSat.h
 *
 * Created: 2015-03-22 19:14:56
 *  Author: stratus
 */ 


#ifndef CANSAT_H_
#define CANSAT_H_

//===================Definicja parametrów pracy=================
#define CPU_clock		32							//czêstotliwoœæ taktowania procesora
#define sampling_rate	25.0						//czêstotliwoœæ próbkowania w Hz
#define sampling_time	1/sampling_rate*1000.0		//czas próbkowania w ms
#define telemetry_rate	10.0							//czêstotliwoœæ telemetrii
#define telemetry_time	1/telemetry_rate*1000.0		//czas telemetrii
#define BUZZER_ONOFF	1							//wy³¹cznik buzzera
#define STARTUP_tele	1							//telemetria wy³¹czona na starcie
#define STARTUP_flash	0							//zapis do pamiêci wy³¹czony na starcie
#define STARTUP_armed	0							//rozbrojony na starcie
#define calibrationCNT	(int)(ceil(2*sampling_rate))				//iloœæ próbek do kalibracji

//----------------------------------Data interfaces--------
#define FLASH_SPI	SPIC
#define SPI_PORT	PORTC
#define FLASH_CS_PIN PIN0_bm
#define FLASH_CS_PORT PORTD	
#define SPI_speed		2000	//prêdkoœæ SPI w kHz

#define	SENSORS_I2C	TWIC
#define I2C_speed		300		//prêdkoœæ interfejsu I2C w kHz

#define XBEE_UART		USARTD0
#define XBEE_UART_PORT	PORTD
#define GPS_UART		USARTF0
#define GPS_UART_PORT	PORTF
#define GPS_SPEED		115200	//115200, 9600

#if GPS_SPEED == 115200
	#define GPS_BSEL	1047
	#define GPS_BSCALE	0b10100000
#elif GPS_SPEED == 9600
	#define GPS_BSEL	3317
	#define GPS_BSCALE	0b11000000
#endif

//----------------------------------Definicja ledów---------
//	LED1 - green	- sensor update
//	LED2 - red		- bad ISR
//	LED3 - blue		- receive XB, remote command
//	LED4 - orange	- GPS
//	LED5 - yellow	- calibration

#define LED1 PIN2_bm
#define LED2 PIN3_bm
#define LED3 PIN4_bm
#define LED4 PIN5_bm
#define LED5 PIN6_bm
#define LED_PORT PORTA

//----------------------------Definicja przycisków-------------------
//	SW1 -			- Read
//	SW2 -			- Bootloader

//#define SW1 
//#define SW2

//---------------------------Definicja parametrów filtrów exp--------------
#define LPS25H_alti_alpha 0.2
#define LPS25H_velo_alpha 0.1
#define MPU_acc_alpha 0.3
#define MPU_velo_alpha 0.3
#define MPU_alti_alpha 0.3
#define MPU_gyro_alpha 0.3
#define MPU_angle_alpha 0.3
#define BAT_voltage_alpha 0.05

//-------------------------Definicje parametrów filtrów komplementarnych----
#define altitude_acc_press_beta	0.1
#define velocity_acc_press_beta	0.1


void USART_SendCache(void);
uint8_t I2C_ReadEnd(bool koniec);
void structInit(void);
void Initialization(void);
void UpdateFW(void);
void TempUpdate(struct DS18B20_t * DS18B20, struct frame_t * frame);
void SensorCal(void);
void BT_Start(struct frame_t * frame);
void FLASHerase(void);
#endif /* CANSAT_H_ */
