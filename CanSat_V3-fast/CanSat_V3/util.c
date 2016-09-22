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

void prepareFrame(struct frame_t *  frame, struct stan_t *  stan, struct GPS_t * gps){
	volatile int16_t i,tmp,tmpf;
	volatile uint32_t tmp_long;
	i=0;
	//-----------------header----------------------------
	frame->frameASCII[i++] = '3';
	frame->frameASCII[i++] = '2';
	frame->frameASCII[i++] = '9';
	frame->frameASCII[i++] = '6';
	frame->frameASCII[i++] = ',';
	//----------------packet count-----------------------
	tmp = frame->r_count;
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	//-------------Bat voltage----------------------
	tmp = (int16_t)(frame->r_voltage*1000);
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	//------------Flight state------------------------
	tmp = frame->r_FSWstate;
	frame->frameASCII[i++] =  tmp+ 48;
	frame->frameASCII[i++] =  ',';
	//------------FSW state-------------------------
	frame->frameASCII[i++] = 'S';
	frame->frameASCII[i++] = stan->armed_trigger+48;
	frame->frameASCII[i++] = stan->telemetry_trigger+48;
	frame->frameASCII[i++] = stan->flash_trigger+48;
	frame->frameASCII[i++] = stan->callibration+48;
	frame->frameASCII[i++] = ',';
	
	//===============Pressure & altitude===================
	//--------------LPS25H Altitude----------------------------
	tmp = frame->LPS25H_altitude;
	tmpf = (frame->LPS25H_altitude - truncf(frame->LPS25H_altitude))*100;
	if((tmp < 0) || (tmpf <0)){
		tmp = -tmp;
		tmpf = -tmpf;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmpf/10)%10 + 48;
	frame->frameASCII[i++] = (tmpf/1)%10 + 48;
	frame->frameASCII[i++] = ',';
	
//------------------LPS25H Velocity------------------------------
	tmp = frame->LPS25H_velocity*10;	
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//===============Accel=========================
	//--------------MPU9150 Accel x----------------------
	tmp = frame->MPU9150_accel_x*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//===============Gyro==========================
	
	//--------------MPU9150 Gyro y---------------------
	tmp = frame->MPU9150_gyro_y*10;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';

	//==============GPS============================
	//--------------GPS lat------------------------------
	frame->frameASCII[i++] = gps->latitude[0];
	frame->frameASCII[i++] = gps->latitude[1];
	frame->frameASCII[i++] = gps->latitude[2];
	frame->frameASCII[i++] = gps->latitude[3];
	frame->frameASCII[i++] = gps->latitude[4];
	frame->frameASCII[i++] = gps->latitude[5];
	frame->frameASCII[i++] = gps->latitude[6];
	frame->frameASCII[i++] = gps->latitude[7];
	frame->frameASCII[i++] = gps->latitude[8];
	frame->frameASCII[i++] = gps->latitude[9];
	frame->frameASCII[i++] = gps->latitude[10];
	frame->frameASCII[i++] = ',';
	//--------------GPS long------------------------------
	frame->frameASCII[i++] = gps->longitude[0];
	frame->frameASCII[i++] = gps->longitude[1];
	frame->frameASCII[i++] = gps->longitude[2];
	frame->frameASCII[i++] = gps->longitude[3];
	frame->frameASCII[i++] = gps->longitude[4];
	frame->frameASCII[i++] = gps->longitude[5];
	frame->frameASCII[i++] = gps->longitude[6];
	frame->frameASCII[i++] = gps->longitude[7];
	frame->frameASCII[i++] = gps->longitude[8];
	frame->frameASCII[i++] = gps->longitude[9];
	frame->frameASCII[i++] = gps->longitude[10];
	frame->frameASCII[i++] = ',';
	//--------------GPS altitude------------------------------
	frame->frameASCII[i++] = gps->altitude[0];
	frame->frameASCII[i++] = gps->altitude[1];
	frame->frameASCII[i++] = gps->altitude[2];
	frame->frameASCII[i++] = gps->altitude[3];
	frame->frameASCII[i++] = gps->altitude[4];
	frame->frameASCII[i++] = gps->altitude[5];
	frame->frameASCII[i++] = gps->altitude[6];
	frame->frameASCII[i++] = gps->altitude[7];
	frame->frameASCII[i++] = gps->altitude[8];
	frame->frameASCII[i++] = ',';
	//--------------GPS fix------------------------------
	//frame->frameASCII[i++] = 'F';
	frame->frameASCII[i++] = gps->fix+48;
	frame->frameASCII[i++] = ',';
	
	//--------------Checksum----------------------------
	frame->frameASCII[i++] = '1';
	frame->frameASCII[i++] = '3';
	
	//--------------Remote end------------------------------
	frame->frameASCII[i++] = ',';
	frame->frameASCII[i++] = '%';
	frame->frameASCII[i++] = '#';
	frame->frameASCII[i++] = ',';
	
	//Local start
	//---------------Time [s]----------------------------
	tmp_long = frame->sec;
	frame->frameASCII[i++] = (tmp_long/1000000)%10 + 48;
	frame->frameASCII[i++] = (tmp_long/100000)%10 + 48;
	frame->frameASCII[i++] = (tmp_long/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp_long/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp_long/100)%10 + 48;
	frame->frameASCII[i++] = (tmp_long/10)%10 + 48;
	frame->frameASCII[i++] = (tmp_long)%10 + 48;
	frame->frameASCII[i++] = ',';
	//-------------Bat voltage----------------------
	tmp = (int16_t)(frame->r_voltage*1000);
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	//-------------VCC voltage----------------------
	tmp = (int16_t)(frame->vcc*1000);
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//===============Temperature=====================
	//--------------MPU9150 temp-----------------------
	tmp = frame->MPU9150_temp*100;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = tmp%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//--------------LSM9DS0 temp----------------------- //93
	/*
	tmp = frame->LSM9DS0_temp*100;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = tmp%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	
	//================Analog================================
	//--------------Analog 1------------------------------
	/*
	tmp = frame->light1;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = '%';
	frame->frameASCII[i++] = ',';
	*/
	//--------------Analog 2------------------------------
	/*
	tmp = frame->light2;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = '%';
	frame->frameASCII[i++] = ',';
	*/
	//--------------Analog 3------------------------------
	/*
	tmp = frame->light3;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = '%';
	frame->frameASCII[i++] = ',';
	*/
	
	//--------------LPS25H pressure----------------------
	tmp = frame->LPS25H_pressure;
	tmpf = ((frame->LPS25H_pressure - truncf(frame->LPS25H_pressure))*1000);
	if(tmp < 0){
		tmp = -tmp;
		tmpf = -tmpf;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp/1)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmpf/100)%10 + 48;
	frame->frameASCII[i++] = (tmpf/10)%10 + 48;
	frame->frameASCII[i++] = (tmpf/1)%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//===============Acceleration=========================
	//--------------MPU9150 Accel y------------------------
	/*
	tmp = frame->MPU9150_accel_y*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------MPU9150 Accel z-------------------------
	/*
	tmp = frame->MPU9150_accel_z*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------LIS331HH Accel x----------------------
	/*
	tmp = frame->LIS331HH_accel_x*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------LIS331HH Accel y------------------------
	tmp = -frame->LIS331HH_accel_y*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//--------------LIS331HH Accel z-------------------------
	/*
	tmp = frame->LIS331HH_accel_z*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------LSM9DS0 Accel x----------------------//124
	/*
	tmp = frame->LSM9DS0_accel_x*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------LS9DS0 Accel y------------------------
	tmp = frame->LSM9DS0_accel_y*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';

	//--------------LSM9DS0 Accel z-------------------------
	/*
	tmp = frame->LSM9DS0_accel_z*1000;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	
	//==========================Gyro======================
	//--------------MPU9150 Gyro x---------------------//156
	
	tmp = frame->MPU9150_gyro_x*10;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//--------------MPU9150 Gyro z----------------------
	
	tmp = frame->MPU9150_gyro_z*10;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	
	//--------------LSM9DS0 Gyro x---------------------
	/*
	tmp = frame->LSM9DS0_gyro_x*10;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------LSM9DS0 Gyro y---------------------
	tmp = frame->LSM9DS0_gyro_y*10;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	//--------------LSM9DS0 Gyro z----------------------//188
	/*
	tmp = frame->LSM9DS0_gyro_z*10;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/10000)%10 + 48;
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//===============Mag===========================	
	/*
	//--------------LSM9DS0 mag x-----------------------//220
	tmp = -frame->LSM9DS0_mag_x*100;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------LSM9DS0 mag y-----------------------
	/*
	tmp = frame->LSM9DS0_mag_y*100;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	//--------------LSM9DS0 mag z-----------------------
	/*
	tmp = frame->LSM9DS0_mag_z*100;
	if(tmp < 0){
		tmp = -tmp;
		frame->frameASCII[i++] = '-';
	}
	else frame->frameASCII[i++] = '+';
	frame->frameASCII[i++] = (tmp/1000)%10 + 48;
	frame->frameASCII[i++] = (tmp/100)%10 + 48;
	frame->frameASCII[i++] = '.';
	frame->frameASCII[i++] = (tmp/10)%10 + 48;
	frame->frameASCII[i++] = (tmp)%10 + 48;
	frame->frameASCII[i++] = ',';
	*/
	
	frame->frameASCII[i++] = '\r';
	frame->frameASCII[i++] = '\n';		//248 bajtów
	frame->frameASCII[i++] = 0;
	frame->frameASCII[i++] = 'X';
}

char ringBuffer_read(struct ringBuffer_t * bufor){
	char tmp;
	if((((*bufor).bufferEnd - (*bufor).bufferStart) != 0) && ((*bufor).mutex == false)){
		tmp = (*bufor).data[(*bufor).bufferStart];
		(*bufor).bufferStart = ((*bufor).bufferStart + 1)%490;
		(*bufor).mutex = false;
	}
	else tmp = 0;
	return tmp;
}

bool ringBuffer_addChar(struct ringBuffer_t * bufor, char value){
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

bool ringBuffer_addString(struct ringBuffer_t * bufor, char * text, uint16_t text_length){
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

bool purgeBuffer(struct ringBuffer_t * bufor){
	(*bufor).bufferEnd = 0;
	return false;
}

bool GPSdecode(struct ringBuffer_t * bufor, struct GPS_t * gps){
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

void decodeNMEA(struct GPS_t * GPS, struct ringBuffer_t * GPSbuf){
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

void altitudeCalc(struct BMP085_t * BMP085, struct frame_t * frame){
	//frame->r_altitude = press;
	//volatile float tmp1, tmp2, tmp3, press, start_press;
	//press = BMP085->pressure;
	//start_press = BMP085->start_pressure;
	//tmp1 = 1/5.255;
	//tmp2 = press/start_press;
	//tmp3 = (1-powf(tmp2, tmp1));
	
	//float old_altitude = frame->r_altitude;
	//frame->r_altitude = (frame->BMP*0.7) + (44330.0*tmp3*0.3);	//Exponential smoothing
	//frame->dif_altitude = frame->dif_altitude*0.9 + (frame->r_altitude-old_altitude)*0.1;
}

void altitudeCalcLPS(struct LPS25H_t * LPS25H, struct frame_t * frame){
	volatile float new_altitude, press, start_press;
	press = LPS25H->pressure;
	start_press = LPS25H->start_pressure;
	new_altitude = (1-pow(press/start_press, 0.1902632365))*43538.0;
	
	float old_altitude = frame->LPS25H_altitude;
	frame->LPS25H_altitude = old_altitude*(1-LPS25H_alti_alpha) + new_altitude*LPS25H_alti_alpha;	//Exponential smoothing
	frame->dif_altitude = frame->dif_altitude*(1-LPS25H_velo_alpha) + (frame->LPS25H_altitude-old_altitude)*LPS25H_velo_alpha;
	frame->LPS25H_velocity = frame->dif_altitude * sampling_rate;
}

void GPSbuf_init(struct GPS_t * gps){
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

void maxAcc(struct frame_t * dane){
	dane->MPU9150_sumacc = abs(dane->MPU9150_accel_x)+abs(dane->MPU9150_accel_x)+abs(dane->MPU9150_accel_x);
	dane->LIS331HH_sumacc = abs(dane->LIS331HH_accel_x)+abs(dane->LIS331HH_accel_x)+abs(dane->LIS331HH_accel_x);
	if(dane->MPU9150_sumacc > dane->LIS331HH_sumacc) dane->max_acc = dane->MPU9150_sumacc;
	else dane->max_acc = dane->LIS331HH_sumacc;
}