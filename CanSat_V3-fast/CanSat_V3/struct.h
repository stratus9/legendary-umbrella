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

//--------------------------struktura obs�ugi czujnika BMP085---------------------------------
struct BMP085_t{
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
};

//-------------------------struktura obs�ugi czujnika MPU9150--------------------------------
struct MPU9150_t{
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
	float accel_sum;		//zmieni� na uint16_t po doko�czeniu funkcji mat
	
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
};

//-------------------------struktura obs�ugi czujnika LSM9DS0--------------------------------
struct LSM9DS0_t{
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
	float accel_sum;		//zmieni� na uint16_t po doko�czeniu funkcji mat
	
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
};
//-----------------------------------------struktura macierzy 3x3-----------------------
struct matrix3x3{
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
};

//----------------------------------struktura obs�ugi okre�lania orientacji------------
struct Orient_t{
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
};

//-------------------------------struktura obs�ugi maszyny stan�w-------------------
struct stan_t{
	bool new_data;
	bool new_frame;
	bool cmd_mode;
	bool callibration;
	bool telemetry_trigger;
	bool flash_trigger;
	bool armed_trigger;
	uint8_t LIS_comm;
};

//-------------------------------struktura obs�ugi interfejsu USART----------------
struct USART_t{
	uint8_t in_i;			//licznik pozycji bufora wej�ciowego
	char in[50];			//bufer wej�ciowy
	bool in_inprogress;		//trwa odbi�r
	bool in_ready;			//odebrano ramk�
	bool in_error;			//b��d odbioru
	
	uint8_t out_i;			//licznik pozycji bufora wej�ciowego
	char out[50];			//bufer wej�ciowy
	bool out_inprogress;	//trwa odbi�r
	bool out_ready;			//odebrano ramk�
	bool out_error;			//b��d odbioru
	
	bool TxFlag;			//sterowanie transmisj�
	bool RxFlag;
};

//---------------------------struktura obs�ugi ADC--------------------------------
struct ADC_t{
	uint16_t ADC16u;
	int16_t ADC16s;
	float Vsense;
	float Vsense2;
	float Vusb;
	float LS1;
	float LS2;
	float LS3;
	float VCC;
	};

//--------------------------obs�uga przycisk�w i diodek---------------------------
struct IO_t{
	bool switch1_c;
	bool switch1_p;
	bool switch2_c;
	bool switch2_p;
	bool LED1;
	bool LED2;
	bool LED3;
	};

//------------------------FRAME--------------------------------------------------
struct frame_t{
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
	//LPS25H	float LPS25H_pressure;
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
};

//---------------------------------Bufor cykliczny-------------
struct ringBuffer_t{
	char data[500];
	uint16_t bufferEnd;
	uint16_t bufferStart;
	uint8_t bufferCount;
	bool bufferFull;
	bool bufferEmpty;
	bool mutex;
	bool block;
	bool datarReady;
};

//---------------------------------GPS-------------------------
struct GPS_t{
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
};

struct DS18B20_t{
	float temp1;
	float temp2;
	uint8_t MSB;
	uint8_t LSB;
	//doda� zmienn� na adresy czujnik�w
	bool data_ready;
	bool request_sent;
};

struct LPS25H_t{
	uint32_t raw_pressure;
	float pressure;
	int16_t raw_temp;
	float temp;
	float start_pressure;
	
};

struct LIS331HH_t{
	int16_t raw_accel_x;
	int16_t raw_accel_y;
	int16_t raw_accel_z;
	float accel_x;
	float accel_y;
	float accel_z;
};

struct Calibration_t{
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
};

struct buzzer_t{
	bool trigger;
	uint8_t mode;
	uint8_t i;
	uint8_t count;
};
		
#endif /* STRUCT_H_ */