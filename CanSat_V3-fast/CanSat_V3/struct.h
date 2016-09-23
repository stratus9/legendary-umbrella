/*
 * struct.h
 *
 * Created: 2015-03-22 16:41:01
 *  Author: stratus
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef STRUCT_H_
#define STRUCT_H_

//--------------------------struktura obs³ugi czujnika BMP085---------------------------------
typedef struct BMP085_s{
	bool data_ready;
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	short md;
	float temperature;
	float pressure;
	uint16_t ut;
	uint16_t up;
	uint8_t state;
	float start_pressure;
} BMP085_t;

//-------------------------struktura obs³ugi czujnika MPU9150--------------------------------
typedef struct MPU9150_s{
	int16_t raw_accel_x;
	int16_t raw_accel_y;
	int16_t raw_accel_z;
	int16_t raw_gyro_x;
	int16_t raw_gyro_y;
	int16_t raw_gyro_z;
	int16_t raw_temp;
	int16_t raw_mag_x;
	int16_t raw_mag_y;
	int16_t raw_mag_z;
	
	float accel_x;
	float accel_y;
	float accel_z;
	float accel_sum;		//zmieniæ na uint16_t po dokoñczeniu funkcji mat
	
	float gyro_x;
	float gyro_y;
	float gyro_z;
	int16_t temp;
	float mag_x;
	float mag_y;
	float mag_z;
	float mag_sum;
	
	int16_t offset_accel_x;
	int16_t offset_accel_y;
	int16_t offset_accel_z;
	int16_t offset_gyro_x;
	int16_t offset_gyro_y;
	int16_t offset_gyro_z;
	int16_t offset_mag_x;
	int16_t offset_mag_y;
	int16_t offset_mag_z;
	int8_t sens_mag_x;
	int8_t sens_mag_y;
	int8_t sens_mag_z;
	uint8_t mag_status;
} MPU9150_t;

//-------------------------struktura obs³ugi czujnika LSM9DS0--------------------------------
typedef struct LSM9DS0_s{
	int16_t raw_accel_x;
	int16_t raw_accel_y;
	int16_t raw_accel_z;
	int16_t raw_gyro_x;
	int16_t raw_gyro_y;
	int16_t raw_gyro_z;
	int16_t raw_temp;
	int16_t raw_mag_x;
	int16_t raw_mag_y;
	int16_t raw_mag_z;
	
	float accel_x;
	float accel_y;
	float accel_z;
	float accel_sum;		//zmieniæ na uint16_t po dokoñczeniu funkcji mat
	
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float temp;
	float mag_x;
	float mag_y;
	float mag_z;
	float mag_sum;
	
	int16_t offset_accel_x;
	int16_t offset_accel_y;
	int16_t offset_accel_z;
	int16_t offset_gyro_x;
	int16_t offset_gyro_y;
	int16_t offset_gyro_z;
	int16_t offset_mag_x;
	int16_t offset_mag_y;
	int16_t offset_mag_z;
	int8_t sens_mag_x;
	int8_t sens_mag_y;
	int8_t sens_mag_z;
	uint8_t mag_status;
} LSM9DS0_t;
//-----------------------------------------struktura macierzy 3x3-----------------------
typedef struct matrix3x3_s{
	float a11;
	float a12;
	float a13;
	float a21;
	float a22;
	float a23;
	float a31;
	float a32;
	float a33;
	float det;
} matrix3x3_t;

//----------------------------------struktura obs³ugi okreœlania orientacji------------
typedef struct Orient_s{
	int32_t x_angle_1000;
	int32_t y_angle_1000;
	int32_t z_angle_1000;
	
	float x_angle;
	float y_angle;
	float z_angle;
	
	int32_t q1;
	int32_t q2;
	int32_t q3;
	
	float dx;
	float dy;
	float dz;
} Orient_t;

//-------------------------------struktura obs³ugi maszyny stanów-------------------
typedef struct stan_s{
	bool new_data;
	bool new_frame;
	bool cmd_mode;
	bool callibration;
	bool telemetry_trigger;
	bool flash_trigger;
	bool armed_trigger;
	uint8_t LIS_comm;
} stan_t;

//-------------------------------struktura obs³ugi interfejsu USART----------------
typedef struct USART_s{
	uint8_t in_i;			//licznik pozycji bufora wejœciowego
	char in[50];			//bufer wejœciowy
	bool in_inprogress;		//trwa odbiór
	bool in_ready;			//odebrano ramkê
	bool in_error;			//b³¹d odbioru
	
	uint8_t out_i;			//licznik pozycji bufora wejœciowego
	char out[50];			//bufer wejœciowy
	bool out_inprogress;	//trwa odbiór
	bool out_ready;			//odebrano ramkê
	bool out_error;			//b³¹d odbioru
	
	bool TxFlag;			//sterowanie transmisj¹
	bool RxFlag;
} USARTdata_t;

//---------------------------struktura obs³ugi ADC--------------------------------
typedef struct ADC_s{
	uint16_t ADC16u;
	int16_t ADC16s;
	float Vsense;
	float Vsense2;
	float Vusb;
	float LS1;
	float LS2;
	float LS3;
	float VCC;
	} Analog_t;

//--------------------------obs³uga przycisków i diodek---------------------------
typedef struct IO_s{
	bool switch1_c;
	bool switch1_p;
	bool switch2_c;
	bool switch2_p;
	bool LED1;
	bool LED2;
	bool LED3;
} IO_t;
	
typedef struct SensorsData_s{
	float accel_x;
	float accel_y;
	float accel_z;
	float axisVelo;
	float ascentVelo;
	float altitude;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float batVoltage;
	float inTemp;
	float outTemp;
	char latitude[13];
	char longitude[13];
	char fix;
	char satNo[3];
	float thrust;	
	uint8_t flightState;
	uint8_t softwareState;	
} SensorsData_t;

typedef struct allData_s{
	
} allData_t;

//------------------------FRAME--------------------------------------------------
typedef struct frame_s{
	char frameASCII[600];
	uint16_t iUART;
	bool mutex;
	//time
	uint16_t sec;
	//required telemetry
	uint32_t r_count;
	float r_voltage;
	float vcc;
	uint8_t r_FSWstate;
	bool terminate;
	float max_acc;
	//GPS
	char latitude[14];
	char longitude[14];
	char altitude[9];
	//MPU9150
	float MPU9150_accel_x;
	float MPU9150_accel_y;
	float MPU9150_accel_z;
	float MPU9150_gyro_x;
	float MPU9150_gyro_y;
	float MPU9150_gyro_z;
	float MPU9150_mag_x;
	float MPU9150_mag_y;
	float MPU9150_mag_z;
	float MPU9150_temp;
	float MPU9150_sumacc;
	//LSM9DS0
	float LSM9DS0_accel_x;
	float LSM9DS0_accel_y;
	float LSM9DS0_accel_z;
	float LSM9DS0_gyro_x;
	float LSM9DS0_gyro_y;
	float LSM9DS0_gyro_z;
	float LSM9DS0_mag_x;
	float LSM9DS0_mag_y;
	float LSM9DS0_mag_z;
	float LSM9DS0_temp;
	//BMP085
	float BMP085_pressure;
	float BMP085_temp;
	//LPS25H
	float LPS25H_pressure;
	float LPS25H_temp;
	float LPS25H_altitude;
	float LPS25H_velocity;
	//HMC5883L	float HMC5883L_mag_x;
	float HMC5883L_mag_y;
	float HMC5883L_mag_z;
	//LIS331HH
	float LIS331HH_accel_x;
	float LIS331HH_accel_y;
	float LIS331HH_accel_z;
	float LIS331HH_sumacc;
	//Light
	uint8_t light1;
	uint8_t light2;
	uint8_t light3;
	//state
	float max_altitude;
	float dif_altitude;
} frame_t;

//---------------------------------Bufor cykliczny-------------
typedef struct ringBuffer_s{
	char data[500];
	uint16_t bufferEnd;
	uint16_t bufferStart;
	uint8_t bufferCount;
	bool bufferFull;
	bool bufferEmpty;
	bool mutex;
	bool block;
	bool datarReady;
} ringBuffer_t;

//---------------------------------GPS-------------------------
typedef struct GPS_s{
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
	uint8_t ms;
	char latitude[14];
	char longitude[14];
	char altitude[10];
	uint8_t satelliteN;
	uint8_t singnal;
	uint8_t fix;
	char buffer[20];
	uint8_t count;
	uint8_t data_count;
	bool valid;
	bool frame_start;
	bool new_data;
	bool frame_ok;
	bool frame_new;
} GPS_t;

typedef struct DS18B20_s{
	float temp1;
	float temp2;
	uint8_t MSB;
	uint8_t LSB;
	//dodaæ zmienn¹ na adresy czujników
	bool data_ready;
	bool request_sent;
} DS18B20_t;

typedef struct LPS25H_s{
	uint32_t raw_pressure;
	float pressure;
	int16_t raw_temp;
	float temp;
	float start_pressure;
	
} LPS25H_t;

typedef struct LIS331HH_s{
	int16_t raw_accel_x;
	int16_t raw_accel_y;
	int16_t raw_accel_z;
	float accel_x;
	float accel_y;
	float accel_z;
} LIS331HH_t;

typedef struct Calibration_s{

	//BMP085
	float BMP_pressure;
	//LPS25H
	float LPS_pressure;
	//MPU9150
	int32_t MPU_gyro_x;
	int32_t MPU_gyro_y;
	int32_t MPU_gyro_z;
	//LSM9DS0
	int32_t LSM_gyro_x;
	int32_t LSM_gyro_y;
	int32_t LSM_gyro_z;
	//counter
	uint16_t counter;
	bool trigger;
} Calibration_t;

typedef struct buzzer_s{
	bool trigger;
	uint8_t mode;
	uint8_t i;
	uint8_t count;
} buzzer_t;
		
#endif /* STRUCT_H_ */