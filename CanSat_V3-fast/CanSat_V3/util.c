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
<<<<<<< HEAD
	sprintf(allData->frame_b->frameASCII,
	"$%06lu,%1u,%03.1f,%+07.1f,%+06.1f,%+06.3f,%+05.2f,%+05.2f,%+06.1f,%+06.1f,%+06.1f,%+06.2f,%+06.2f,%+06.2f,%s,%s,%s,%1u\r\n\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
=======
	uint16_t length = 0;
	length = sprintf((char*)allData->frame_b->frameASCII,
	"$SpaceForest,%06lu,%1u,%04.2f,%+08.2f,%+06.1f,%+06.2f,%+06.2f,%+06.2f,%+06.1f,%+06.1f,%+06.1f,%+09.5f,%+010.5f,%+08.1f,%1u\r\n",
>>>>>>> origin/master
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
<<<<<<< HEAD
	allData->SensorsData->mag_x,		//mag w osi rakiety w deg/s
	allData->SensorsData->mag_y,		//mag w osi Y rakiety w deg/s
	allData->SensorsData->mag_z,		//mag w osi Z rakiety w deg/s
	allData->GPS->latitude,
	allData->GPS->longitude,
	allData->GPS->altitude,
	allData->GPS->fix);
	
	
=======
	allData->GPS->lat/10000000.0,
	allData->GPS->lon/10000000.0,
	allData->GPS->alti/10.0,
	allData->GPS->fix);
	
	allData->frame_b->length = length;
>>>>>>> origin/master
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
	gps->altitude[8] = 0;
	gps->altitude[9] = 0;

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

void GPS_toNumber(GPS_t * GPS) {
	CHAR buf[12];
	// decode Latitude from char to num
	buf[0] = GPS->latitude[1];
	buf[1] = GPS->latitude[2];
	buf[2] = 0;
	uint8_t lat1 = (buf[0]-48)*10 + (buf[1]-48);	//stopnie
	
	buf[0] = GPS->latitude[3];
	buf[1] = GPS->latitude[4];
	buf[2] = GPS->latitude[6];
	buf[3] = GPS->latitude[7];
	buf[4] = GPS->latitude[8];
	buf[5] = GPS->latitude[9];
	buf[6] = GPS->latitude[10];
	buf[7] = 0;
	
	uint32_t lat2 = atol((char*)buf);					//minuty*100000
	lat2 /= 0.6f;
	
	int32_t lat = lat1*10000000 + lat2;
	if(GPS->latitude[0] == 'N') lat = lat;
	else if (GPS->latitude[0] == 'S') lat = -lat;
	else lat = 0;
	
	// decode Longitude from char to num
	buf[0] = GPS->longitude[1];
	buf[1] = GPS->longitude[2];
	buf[2] = GPS->longitude[3];
	buf[3] = 0;
	uint8_t lon1 = atoi((char*)buf);	//stopnie
	
	buf[0] = GPS->longitude[4];
	buf[1] = GPS->longitude[5];
	buf[2] = GPS->longitude[7];
	buf[3] = GPS->longitude[8];
	buf[4] = GPS->longitude[9];
	buf[5] = GPS->longitude[10];
	buf[6] = GPS->longitude[11];
	buf[7] = 0;
	
	uint32_t lon2 = atol((char*)buf);					//minuty*100000
	lon2 /= 0.6f;
	
	int32_t lon = lon1*10000000 + lon2;
	if(GPS->longitude[0] == 'E') lon = lon;
	else if (GPS->longitude[0] == 'W') lon = -lon;
	else lon = 0;
	
	// decode Altiutde from char to num
	/* TODO */
	float alti = atof((char*)GPS->altitude);
	
// 	// przepisanie do struktury GPS
	GPS->lat = lat;
 	GPS->lon = lon;
 	GPS->alti = (int32_t)(alti*10.0);	//w mm
}