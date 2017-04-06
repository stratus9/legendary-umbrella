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

void float2char(float number,char * tablica){
	uint16_t tmp;
	if(number < 0){
		*(tablica) = '-';
		tmp = 10*(-number);
	}
	else{
		*(tablica) = '+';
		tmp = 10*number;
	}
	*(tablica+1) = ((tmp%100000UL)/10000UL) + 48;
	*(tablica+2) = ((tmp%10000UL)/1000UL) + 48;
	*(tablica+3) = ((tmp%1000UL)/100) + 48;
	*(tablica+4) = ((tmp%100)/10) + 48;
	*(tablica+5) = '.';
	*(tablica+6) = ((tmp%10)) + 48;
}

void prepareFrame(allData_t * allData){
	sprintf(allData->frame->frameASCII,
	"$AGHSS,%06lu,%1u,%01.2f,%+05.2f,%+04.1f,%+02.3f,%+04.1f,%s,%s,%s,%1u\r\n",
	allData->RTC->time,					//czas w ms
	allData->stan->flightState,			//faza lotu
	allData->Analog->Vbat,				//napiêcie baterii
	allData->SensorsData->altitude,		//wysokoœæ lotu w m
	allData->SensorsData->ascentVelo,	//prêdkoœæ wznoszenia w m/s
	allData->SensorsData->accel_x,		//przyspieszenie w osi rakiety w g
	allData->SensorsData->gyro_x,		//prêdkoœæ obrotu w osi rakiety w deg/s
	allData->GPS->latitude,
	allData->GPS->longitude,
	allData->GPS->altitude,
	allData->GPS->fix);
}

char ringBuffer_read(ringBuffer_t * bufor){
	char tmp;
	if((((*bufor).bufferEnd - (*bufor).bufferStart) != 0) && ((*bufor).mutex == false)){
		tmp = (*bufor).data[(*bufor).bufferStart];
		(*bufor).bufferStart = ((*bufor).bufferStart + 1)%490;
		(*bufor).mutex = false;
	}
	else tmp = 0;
	return tmp;
}

bool ringBuffer_addChar(ringBuffer_t * bufor, char value){
	if(((*bufor).bufferFull == false) && ((*bufor).mutex == false)){
		(*bufor).mutex = true;
		(*bufor).data[(*bufor).bufferEnd] = value;
		(*bufor).bufferEnd = ((*bufor).bufferEnd + 1)%490;
		if((*bufor).bufferEnd >= 495) (*bufor).bufferFull = true;
		(*bufor).mutex = false;
		return false;
	}
	else return true;
}

bool ringBuffer_addString(ringBuffer_t * bufor, char * text, uint16_t text_length){
	if(((*bufor).bufferFull == false) && ((text_length + (*bufor).bufferFull) < 495) && ((*bufor).mutex == false)){
		(*bufor).mutex = true;
		while(*text){
			ringBuffer_addChar(bufor,(*text++));
		}
		(*bufor).mutex = true;
		return false;
	}
	else return true;
}

bool purgeBuffer(ringBuffer_t * bufor){
	(*bufor).bufferEnd = 0;
	return false;
}

bool GPSdecode(ringBuffer_t * bufor, GPS_t * gps){
	/*
	//first parese
	int i=0;
	while(((*bufor).data[i] != '*') && (i < 200)) i++;
	if(((*bufor).data[i+1] > 47) && ((*bufor).data[i+1] < 58)) (*gps).checksum = ((*bufor).data[i+1] - 48) << 4;
	else if(((*bufor).data[i+1] > 64) && ((*bufor).data[i+1] < 71)) (*gps).checksum = ((*bufor).data[i+1] - 55) << 4;
	if(((*bufor).data[i+2] > 47) && ((*bufor).data[i+2] < 58)) (*gps).checksum += (*bufor).data[i+2] - 48;
	else if(((*bufor).data[i+2] > 64) && ((*bufor).data[i+2] < 71)) (*gps).checksum += (*bufor).data[i+2] - 55;

	//checksum check
	if((*gps).checksum == NMEAchecksum((*bufor).data)){
		(*gps).frame_ok = true;
		(*gps).frame_new = true;
	}
	else (*gps).frame_ok = false;
	(*gps).frame_new = true;		//wywaliæ!!!!!!!!!!!!
	*/
	return false;
}

int NMEAchecksum(char *s) {
	int c = 0;
	s++;	//pominiêcie pierwszego znaku $
	while((*s) && ((*s) != '*'))
	c ^= *s++;
	return c;
}

void decodeNMEA(GPS_t * GPS, ringBuffer_t * GPSbuf){
	/*
	uint8_t i = 0;
	uint16_t tmp = 0;
	i = 1;
	while((*GPSbuf).data[i] != ','){
		tmp += (*GPSbuf).data[i];
		i++;
	}
	i++;
	if(tmp == 358){
		//-----------------dekodowanie czasu---------------------
		if((*GPSbuf).data[i] == ',') i++;	//dekoduj kolejn¹ ramkê
		else if((*GPSbuf).data[i+6] == '.'){
			(*GPS).hh = ((*GPSbuf).data[i]-48)*10+((*GPSbuf).data[i+1]-48);
			i = i+2;
			(*GPS).mm = ((*GPSbuf).data[i]-48)*10+((*GPSbuf).data[i+1]-48);
			i = i+2;
			(*GPS).ss = ((*GPSbuf).data[i]-48)*10+((*GPSbuf).data[i+1]-48);
			i = i+2;
			i++;	//pominiêcie przecinka
			(*GPS).ms = ((*GPSbuf).data[i]-48)*100+((*GPSbuf).data[i+1]-48)*10;
			i = i+2;
		}
		//---------------dekodowanie szerokoœci geo---------------
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+1] == ',')) i = i+2;	//dekoduj kolejn¹ ramkê
		else if((*GPSbuf).data[i+5] == '.'){
			i++;
			(*GPS).latitude = ((*GPSbuf).data[i]-48)*1000+((*GPSbuf).data[i+1]-48)*100+(((*GPSbuf).data[i+2]-48)*100+((*GPSbuf).data[i+3]-48)*10)/6;
			i = i+5;
			(*GPS).latitude += ((*GPSbuf).data[i]-48)/10+((*GPSbuf).data[i+1]-48)/100+((*GPSbuf).data[i+2]-48)/1000+((*GPSbuf).data[i+3]-48)/10000+((*GPSbuf).data[i+4]-48)/100000;
			i = i+5;
		}
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+2] == ',')){
			i++;
			if((*GPSbuf).data[i] == 'N') (*GPS).latitude = (*GPS).latitude;
			else if((*GPSbuf).data[i] == 'S') (*GPS).latitude = -(*GPS).latitude;
			else (*GPS).latitude = 0;
			i++;
		}
		
		//------------dekodowanie d³ugoœci geo----------------
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+1] == ',')) i = i+2;	//dekoduj kolejn¹ ramkê
		else if((*GPSbuf).data[i+6] == '.'){
			i++;
			(*GPS).longitude = ((*GPSbuf).data[i]-48)*10000+((*GPSbuf).data[i+1]-48)*1000+((*GPSbuf).data[i+1]-48)*100+(((*GPSbuf).data[i+2]-48)*100+((*GPSbuf).data[i+3]-48)*10)/6;
			i = i+5;
			(*GPS).longitude += ((*GPSbuf).data[i]-48)/10+((*GPSbuf).data[i+1]-48)/100+((*GPSbuf).data[i+2]-48)/1000+((*GPSbuf).data[i+3]-48)/10000+((*GPSbuf).data[i+4]-48)/100000;
			i = i+5;
		}
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+2] == ',')){
			i++;
			if((*GPSbuf).data[i] == 'E') (*GPS).longitude = (*GPS).longitude;
			else if((*GPSbuf).data[i] == 'W') (*GPS).longitude = -(*GPS).longitude;
			else (*GPS).longitude = 0;
			i++;
		}
		
		//-------------dekodowanie rodzaju FIXa------------
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+1] == ',')) i = i+2;	//dekoduj kolejn¹ ramkê
		else if((*GPSbuf).data[i+1] == '1') (*GPS).fix = 1;
		else if((*GPSbuf).data[i+1] == '2') (*GPS).fix = 2;
		else (*GPS).fix = 0;
		i = i+2;
		
		//------------u¿ywane satelity--------------------
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+1] == ',')) i = i+2;	//dekoduj kolejn¹ ramkê
		else if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+2] == ',')){
			(*GPS).satelliteN = (*GPSbuf).data[i+1]-48;
			i = i+2;
		}
		else if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+3] == ',')){
			(*GPS).satelliteN = ((*GPSbuf).data[i+1]-48)*10+(*GPSbuf).data[i+2]-48;
			i = i+3;
		}
		
		//------------dekodowanie dok³adnoœci---------------------
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+1] == ',')) i = i+2;	//dekoduj kolejn¹ ramkê
		else{
			i++;
			(*GPS).accuracy = 0;
			while(((*GPSbuf).data[i] == '.') || ((*GPSbuf).data[i] == ',')){
				(*GPS).accuracy = (*GPS).accuracy*10+(*GPSbuf).data[i]-48;
				i++;
			}
			i++;	//pominiêcie kropki
			if((*GPSbuf).data[i] != ','){
				(*GPS).accuracy += ((*GPSbuf).data[i]-48)/10;
				i++;
				if((*GPSbuf).data[i] != ','){
					(*GPS).accuracy += ((*GPSbuf).data[i]-48)/100;
					i++;
				}
			}	
		}
		
		//-----------dekodowanie wysokoœci na poziomem morza-------
		if(((*GPSbuf).data[i] == ',') && ((*GPSbuf).data[i+1] == ',')) i = i+2;	//dekoduj kolejn¹ ramkê
		else{
			i++;
			(*GPS).altitude = 0;
			while(((*GPSbuf).data[i] == '.') || ((*GPSbuf).data[i] == ',')){
				(*GPS).altitude = (*GPS).altitude*10+(*GPSbuf).data[i]-48;
				i++;
			}
			i++;	//pominiêcie kropki
			if((*GPSbuf).data[i] != ','){
				(*GPS).altitude += ((*GPSbuf).data[i]-48)/10;
				i++;
				if((*GPSbuf).data[i] != ','){
					(*GPS).altitude += ((*GPSbuf).data[i]-48)/100;
					i++;
				}
			}	
		}
		
		//----------------koniec sensownego dekodowania----------
	}
 	(*GPSbuf).mutex = false;
	 */
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