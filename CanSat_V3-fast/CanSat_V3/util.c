/*
 * util.c
 *
 * Created: 2015-03-22 20:23:40
 *  Author: stratus
 */ 
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "struct.h"
#include "util.h"
#include "CanSat.h"

void prepareFrame(allData_t * allData){
	uint16_t length = 0;
	length = sprintf((char*)allData->frame_b->frameASCII,
	"$SpaceForest,%06lu,%1u,%04.2f,%+08.2f,%+06.1f,%+06.3f,%+06.3f,%+06.3f,%+06.1f,%+06.1f,%+06.1f,%s,%s,%s,%1u\r\n",
	allData->RTC->time,					//czas w ms
	allData->stan->flightState,			//faza lotu
	allData->Analog->Vbat,				//napiêcie baterii
	allData->SensorsData->altitude,		//wysokoœæ lotu w m
	allData->SensorsData->ascentVelo,	//prêdkoœæ wznoszenia w m/s
	allData->SensorsData->accel_x,		//przyspieszenie w osi rakiety w g
	allData->SensorsData->accel_y,		//przyspieszenie w osi Y rakiety w g
	allData->SensorsData->accel_z,		//przyspieszenie w osi Z rakiety w g
	allData->SensorsData->gyro_x,		//prêdkoœæ obrotu w osi rakiety w deg/s
	allData->SensorsData->gyro_y,		//prêdkoœæ obrotu w osi Y rakiety w deg/s
	allData->SensorsData->gyro_z,		//prêdkoœæ obrotu w osi Z rakiety w deg/s
	allData->GPS->latitude,
	allData->GPS->longitude,
	allData->GPS->altitude,
	allData->GPS->fix);
	
	allData->frame_b->length = length;
}



bool purgeBuffer(ringBuffer_t * bufor){
	(*bufor).bufferEnd = 0;
	return false;
}

int NMEAchecksum(CHAR *s) {
	int c = 0;
	s++;	//pominiêcie pierwszego znaku $
	while((*s) && ((*s) != '*'))
	c ^= *s++;
	return c;
}

void GPSbuf_init(GPS_t * gps){
	gps->altitude[0] = '0';
	gps->altitude[1] = '0';
	gps->altitude[2] = '0';
	gps->altitude[3] = '0';
	gps->altitude[4] = '0';
	gps->altitude[5] = '0';
	gps->altitude[6] = '0';
	gps->altitude[7] = '0';
	gps->altitude[8] = '0';
	gps->altitude[9] = '0';

	gps->longitude[0] = '0';
	gps->longitude[1] = '0';
	gps->longitude[2] = '0';
	gps->longitude[3] = '0';
	gps->longitude[4] = '0';
	gps->longitude[5] = '0';
	gps->longitude[6] = '0';
	gps->longitude[7] = '0';
	gps->longitude[8] = '0';
	gps->longitude[9] = '0';
	gps->longitude[10] = '0';
	gps->longitude[11] = '0';
	gps->longitude[12] = '0';
	
	gps->latitude[0] = '0';
	gps->latitude[1] = '0';
	gps->latitude[2] = '0';
	gps->latitude[3] = '0';
	gps->latitude[4] = '0';
	gps->latitude[5] = '0';
	gps->latitude[6] = '0';
	gps->latitude[7] = '0';
	gps->latitude[8] = '0';
	gps->latitude[9] = '0';
	gps->latitude[10] = '0';
	gps->latitude[11] = '0';
	gps->latitude[12] = '0';
	
	gps->fix = 0;
}

float MinAngleVector3D(float x, float y, float z){
	float dot, length,cosa, angle1, angle2, angle3;
	//-----assume reference [1, 0 ,0]-------------------------
	dot = fabs(x);
	length = VectorLength3D(x,y,z);
	cosa = dot/length;
	angle1 = acos(cosa)/3.14*180.0;
	//-----assume reference [0, 1 ,0]-------------------------
	dot = fabs(y);
	length = VectorLength3D(x,y,z);
	cosa = dot/length;
	angle2 = acos(cosa)/3.14*180.0;
	//-----assume reference [0, 0 ,1]-------------------------
	dot = fabs(z);
	length = VectorLength3D(x,y,z);
	cosa = dot/length;
	angle3 = acos(cosa)/3.14*180.0;
	//-----compare--------------------------------------------
	if((angle1 <= angle2) && (angle1 <= angle3)) return angle1;
	else if((angle2 <= angle1) && (angle2 <= angle3)) return angle2;
	else if((angle3 <= angle2) && (angle3 <= angle1)) return angle3;
	else return 0;
}



volatile uint8_t xbee_DMA_ready = 1;


void UART_Xbee_DMA_transfer_nonblocking_start(uint8_t * source, uint16_t length) {
	DMA.CH0.SRCADDR0    =   (uint16_t)source & 0xFF;			// adres Ÿród³a
	DMA.CH0.SRCADDR1    =   (uint16_t)source >> 8;
	DMA.CH0.SRCADDR2    =   0;
	
	DMA.CH0.DESTADDR0   =   (uint16_t)&XBEE_UART.DATA & 0xFF;    // adres celu
	DMA.CH0.DESTADDR1   =   (uint16_t)&XBEE_UART.DATA >> 8;
	DMA.CH0.DESTADDR2   =   0;
	
	DMA.CH0.TRFCNT      =   length;							// rozmiar bloku
	DMA.CH0.TRIGSRC     =   DMA_CH_TRIGSRC_USARTD0_DRE_gc;  // kana³ CH0 powoduje transfer
	DMA.CH0.ADDRCTRL    =   DMA_CH_SRCRELOAD_BLOCK_gc|      // prze³adowanie adresu Ÿród³a po zakoñczeniu bloku
							DMA_CH_SRCDIR_INC_gc|           // zwiêkszanie adresu Ÿród³a po ka¿dym bajcie
							DMA_CH_DESTRELOAD_NONE_gc|      // prze³adowanie adresu celu nigdy
							DMA_CH_DESTDIR_FIXED_gc;        // sta³y adres docelowy
	DMA.CH0.CTRLB		=	DMA_CH_TRNIF_bm;				// Skasuj flagê zakoñczenia transferu 
	DMA.CH0.CTRLA       =   DMA_CH_ENABLE_bm|               // w³¹czenie kana³u
							DMA_CH_BURSTLEN_1BYTE_gc|       // burst = 1 bajt
							DMA_CH_SINGLE_bm;               // pojedynczy burst po ka¿dym zdarzeniu
}

void UART_Xbee_DMA_transfer_blocking_start(uint8_t * source, uint16_t length) {
	DMA.CH0.SRCADDR0    =   (uint16_t)source & 0xFF;			// adres Ÿród³a
	DMA.CH0.SRCADDR1    =   (uint16_t)source >> 8;
	DMA.CH0.SRCADDR2    =   0;
	
	DMA.CH0.DESTADDR0   =   (uint16_t)&XBEE_UART.DATA & 0xFF;    // adres celu
	DMA.CH0.DESTADDR1   =   (uint16_t)&XBEE_UART.DATA >> 8;
	DMA.CH0.DESTADDR2   =   0;
	
	DMA.CH0.TRFCNT      =   length;							// rozmiar bloku
	DMA.CH0.TRIGSRC     =   DMA_CH_TRIGSRC_USARTD0_DRE_gc;  // kana³ CH0 powoduje transfer
	DMA.CH0.ADDRCTRL    =   DMA_CH_SRCRELOAD_BLOCK_gc|      // prze³adowanie adresu Ÿród³a po zakoñczeniu bloku
							DMA_CH_SRCDIR_INC_gc|           // zwiêkszanie adresu Ÿród³a po ka¿dym bajcie
							DMA_CH_DESTRELOAD_NONE_gc|      // prze³adowanie adresu celu nigdy
							DMA_CH_DESTDIR_FIXED_gc;        // sta³y adres docelowy
	DMA.CH0.CTRLB		=	DMA_CH_TRNIF_bm;				// Skasuj flagê zakoñczenia transferu
	DMA.CH0.CTRLA       =   DMA_CH_ENABLE_bm|               // w³¹czenie kana³u
							DMA_CH_BURSTLEN_1BYTE_gc|       // burst = 1 bajt
							DMA_CH_SINGLE_bm;               // pojedynczy burst po ka¿dym zdarzeniu
	
	while (!(DMA.CH0.CTRLB & DMA_CH_TRNIF_bm)) {}
}

uint8_t UART_Xbee_DMA_transfer_nonblocking_ready() {
	return DMA.CH0.CTRLB & DMA_CH_TRNIF_bm;
}


// void SPI_Flash_DMA_transfer_nonblocking_start(uint8_t * source, uint16_t length) {
// 	DMA.CH1.SRCADDR0    =   (uint16_t)source & 0xFF;			// adres Ÿród³a
// 	DMA.CH1.SRCADDR1    =   (uint16_t)source >> 8;
// 	DMA.CH1.SRCADDR2    =   0;
// 	
// 	DMA.CH1.DESTADDR0   =   (uint16_t)&SPIC.DATA & 0xFF;    // adres celu
// 	DMA.CH1.DESTADDR1   =   (uint16_t)&SPIC.DATA >> 8;
// 	DMA.CH1.DESTADDR2   =   0;
// 	
// 	DMA.CH1.TRFCNT      =   length;							// rozmiar bloku
// 	DMA.CH1.TRIGSRC     =   DMA_CH_TRIGSRC_SPIC_gc;			// kana³ CH1 powoduje transfer
// 	DMA.CH1.ADDRCTRL    =   DMA_CH_SRCRELOAD_BLOCK_gc|      // prze³adowanie adresu Ÿród³a po zakoñczeniu bloku
// 	DMA_CH_SRCDIR_INC_gc|           // zwiêkszanie adresu Ÿród³a po ka¿dym bajcie
// 	DMA_CH_DESTRELOAD_NONE_gc|      // prze³adowanie adresu celu nigdy
// 	DMA_CH_DESTDIR_FIXED_gc;        // sta³y adres docelowy
// 	DMA.CH1.CTRLB		=	DMA_CH_TRNIF_bm;				// Skasuj flagê zakoñczenia transferu
// 	DMA.CH1.CTRLA       =   DMA_CH_ENABLE_bm|               // w³¹czenie kana³u
// 	DMA_CH_BURSTLEN_1BYTE_gc|       // burst = 1 bajt
// 	DMA_CH_SINGLE_bm;               // pojedynczy burst po ka¿dym zdarzeniu
// }
// 
// void SPI_Flash_DMA_transfer_blocking_start(uint8_t * source, uint16_t length) {
// 	DMA.CH1.CTRLA		=	DMA_CH_RESET_bm;
// 	
// 	DMA.CH1.SRCADDR0    =   (uint16_t)source & 0xFF;			// adres Ÿród³a
// 	DMA.CH1.SRCADDR1    =   (uint16_t)source >> 8;
// 	DMA.CH1.SRCADDR2    =   0;
// 	
// 	DMA.CH1.DESTADDR0   =   (uint16_t)&SPIC.DATA & 0xFF;    // adres celu
// 	DMA.CH1.DESTADDR1   =   (uint16_t)&SPIC.DATA >> 8;
// 	DMA.CH1.DESTADDR2   =   0;
// 	
// 	DMA.CH1.TRFCNT      =   length;							// rozmiar bloku
// 	DMA.CH1.TRIGSRC     =   DMA_CH_TRIGSRC_SPIC_gc;			// kana³ CH1 powoduje transfer
// 	DMA.CH1.ADDRCTRL    =   DMA_CH_SRCRELOAD_BLOCK_gc|      // prze³adowanie adresu Ÿród³a po zakoñczeniu bloku
// 							DMA_CH_SRCDIR_INC_gc|           // zwiêkszanie adresu Ÿród³a po ka¿dym bajcie
// 							DMA_CH_DESTRELOAD_NONE_gc|      // prze³adowanie adresu celu nigdy
// 							DMA_CH_DESTDIR_FIXED_gc;        // sta³y adres docelowy
// 	DMA.CH1.CTRLB		=	DMA_CH_TRNIF_bm;				// Skasuj flagê zakoñczenia transferu
// 	DMA.CH1.CTRLA       =   DMA_CH_ENABLE_bm|               // w³¹czenie kana³u
// 							DMA_CH_BURSTLEN_1BYTE_gc|       // burst = 1 bajt
// 							DMA_CH_SINGLE_bm;               // pojedynczy burst po ka¿dym zdarzeniu
// 	
// 	while (!(DMA.CH1.CTRLB & DMA_CH_TRNIF_bm)) {}
// }
// 
// uint8_t SPI_Flash_DMA_transfer_nonblocking_ready() {
// 	return DMA.CH1.CTRLB & DMA_CH_TRNIF_bm;
// }