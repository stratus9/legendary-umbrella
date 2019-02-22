/*
 * CanSat_V3.c
 *
 * Created: 2015-03-22 16:38:24
 *  Author: stratus
 */


#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>
#include "Initialization.h"
#include "struct.h"
#include "CanSat.h"
#include "MPU9150.h"
#include "util.h"
#include "LPS25H.h"
#include "LIS331HH.h"
#include "LSM9DS0.h"
#include "SPI.h"
#include "ADC.h"
#include "I2C.h"

//--------TO DO!!!!-----------
//	+GPS
//	+I2C
//	+LED
//	-RTC -zmieniono na timer F
//	-DRY
//	-IO
//	+Buzzer
//	-separacja



//-----------------------------------Struktury globalne---------------------------------------------
static SensorsData_t SensorData_d;
static SensorsData_t SensorData_b;
static Inertial_t Inertial_d;
static allData_t allData_d;
static boardOrient_t boardOrient_d;
static RTC_t RTC_d;
static MPU9150_t MPU9150_d;
static LSM9DS0_t LSM9DS0_d;
static stan_t stan_d;
Analog_t ADC_d;
static frame_t frame_d;
static frame_t frame_b;
static GPS_t GPS_b;
static GPS_t GPS_d;
static LPS25H_t LPS25H_d;
static LIS331HH_t LIS331HH_d;
static Calibration_t Calibration_d;
static buzzer_t buzzer_d;
static FLASH_pageStruct_t FLASH_pageStruct_d;	//strukutra magazynuj�ce dane dla Flash
static uint32_t SPIaddress = 0;
static float timer_buffer = 0;
uint32_t mission_time = 0;
uint32_t frame_count = 0;


//----------------------Bad ISR handling------------------------
ISR(BADISR_vect) {
    LED_PORT.OUTTGL = LED2;
	//asm("nop");
}

//----------------------RTC ISR handling------------------------
/*
ISR(RTC_OVF_vect){
	frame_b.sec++;
	mission_time = frame_b.sec;
}*/

//----------------------Receive from GPS-------------------------
ISR(USARTF0_RXC_vect) {
    uint8_t i = 0;
    asm volatile("nop");
    volatile char tmp = GPS_UART.DATA;
    if(tmp == '$') {
        GPS_b.frame_start = true;
        GPSbuf_init(&GPS_b);
        GPS_b.count = 0;
        GPS_b.data_count = 0;
    } else if(tmp == '*') {
        GPS_b.frame_start = false;
        GPS_b.count = 0;
    } else if(GPS_b.frame_start) {
        //buforowanie fragment�w ramki
        if(tmp == ',') {
            GPS_b.buffer[GPS_b.count++] = ',';
            GPS_b.count = 0;
            GPS_b.data_count++;
            GPS_b.new_data = true;
        } else GPS_b.buffer[GPS_b.count++] = tmp;
        //dekodowanie kolejnych p�l
        if(GPS_b.new_data) {
            GPS_b.new_data = false;
            switch(GPS_b.data_count) {
            case 1:	//typ ramki GPGGA
                //if((GPS_d.buffer[0] != 'G') || (GPS_d.buffer[1] != 'P') || (GPS_d.buffer[2] != 'G') || (GPS_d.buffer[3] != 'G') || (GPS_d.buffer[4] != 'A')) GPS_d.frame_start = false;
                break;
            case 2: //czas UTC
                asm volatile("nop");
                break;
            case 3: //Latitude
                i = 0;
                while((GPS_b.buffer[i] != ',') && (i < 14)) {
                    GPS_b.latitude[i + 1] = GPS_b.buffer[i];
                    i++;
                }
                break;
            case 4: //Latitude
                if(GPS_b.buffer[0] == 'N') GPS_b.latitude[0] = 'N';
                else if(GPS_b.buffer[0] == 'S') GPS_b.latitude[0] = 'S';
                else GPS_b.latitude[0] = 'X';
                break;
            case 5: //Longitude
                i = 0;
                while((GPS_b.buffer[i] != ',') && (i < 14)) {
                    GPS_b.longitude[i + 1] = GPS_b.buffer[i];
                    i++;
                }
                break;
            case 6: //Longitude
                if(GPS_b.buffer[0] == 'E') GPS_b.longitude[0] = 'E';
                else if(GPS_b.buffer[0] == 'W') GPS_b.longitude[0] = 'W';
                else GPS_b.longitude[0] = 'X';
                break;
            case 7:
                if(GPS_b.buffer[0] == '0') GPS_b.fix = 0;
                else if(GPS_b.buffer[0] == '1') GPS_b.fix = 1;
                else if(GPS_b.buffer[0] == '2') GPS_b.fix = 2;
                break;
            case 8:	//sat in use
                asm volatile("nop");
                break;
            case 9: //HDOP
                asm volatile("nop");
                break;
            case 10:	//altitude
                i = 0;
                while((GPS_b.buffer[i] != ',') && (i < 9)) {
                    GPS_b.altitude[i] = GPS_b.buffer[i];
                    i++;
                }
                GPS_d = GPS_b;	//przepisanie danych z bufora
                break;
            default:
                break;
            }
        }
    }
}

//----------------------Send to Xbee--------------------------------
ISR(USARTD0_TXC_vect) {
    if(frame_d.frameASCII[frame_d.iUART]) {
		frame_d.mutex = true;
        XBEE_UART.DATA = frame_d.frameASCII[frame_d.iUART];
        if(frame_d.iUART < 200) frame_d.iUART++;
        else frame_d.frameASCII[frame_d.iUART] = 0;
    } else frame_d.mutex = false;
}

//----------------------Receive from Xbee----------------------------
ISR(USARTD0_RXC_vect) {
    LED_PORT.OUTSET = LED3;
    char volatile tmp = XBEE_UART.DATA;
    if(tmp == '$') stan_d.cmd_mode = true;	//enter command mode
    else if((tmp == 'R') && stan_d.cmd_mode) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            stan_d.cmd_mode = false;
            CPU_CCP = CCP_IOREG_gc;
            RST.CTRL = RST_SWRST_bm;		//zdalny restart systemu
        }
    } else if((tmp == 'C') && stan_d.cmd_mode) {
        Calibration_d.counter = 0;
        Calibration_d.trigger = true;		//zdalna kalibracja czujnik�w
        stan_d.callibration = true;
        stan_d.cmd_mode = false;
    } else if((tmp == 'P') && stan_d.cmd_mode) {
        SPI_ChipErase();					//zerowanie pami�ci FLASH
        SPIaddress = 0;
        stan_d.cmd_mode = false;
        buzzer_d.mode = 1;								//3 sygna�y ci�g�e
        buzzer_d.trigger = true;						//odblokowanie buzzera
		_delay_ms(1000);
    } else if((tmp == 'X') && stan_d.cmd_mode) {
        stan_d.flash_trigger = true;					//start zapisu do pami�ci FLASH
        stan_d.cmd_mode = false;
    } else if((tmp == 'K') && stan_d.cmd_mode) {
        stan_d.flash_trigger = false;					//koniec zapisu do pami�ci FLASH
        stan_d.cmd_mode = false;
    } else if((tmp == 'S') && stan_d.cmd_mode) {
        stan_d.telemetry_trigger = true;			//rozpocz�cie telemetrii
        stan_d.cmd_mode = false;
    } else if((tmp == 'F') && stan_d.cmd_mode) {
        stan_d.telemetry_trigger = false;			//koniec telemetrii
        stan_d.cmd_mode = false;
    } else if((tmp == 'A') && stan_d.cmd_mode) {
        stan_d.armed_trigger = true;				//uzbrojony
        stan_d.cmd_mode = false;
    } else if((tmp == 'D') && stan_d.cmd_mode) {
        stan_d.armed_trigger = false;				//rozbrojony
        stan_d.cmd_mode = false;
    } else stan_d.cmd_mode = false;
}

//----------------------Sensors update-------------------------------
ISR(TCC0_OVF_vect) {
    allData_d.stan->new_data = true;
    allData_d.RTC->time += sampling_time;
}

//----------------------Buzzer---------------------------------------
ISR(TCD0_OVF_vect) {
    if(buzzer_d.trigger && BUZZER_ONOFF) {
        switch(buzzer_d.mode) {
        case 0:
            buzzer_d.trigger = false;
            buzzer_d.count = 0;
            buzzer_d.i = 0;
            PORTF.OUTCLR = PIN6_bm;
            break;
		//-----------3x beep------------
        case 1:
            if(buzzer_d.count > 5) buzzer_d.mode = 0;	//wy��czenie po 3 sygna�ach
            else if(buzzer_d.count <= 5) {
                if(!(buzzer_d.i%3)){
	                PORTF.OUTTGL = PIN6_bm;
	                buzzer_d.count++;
                }
                buzzer_d.i++;
            }
            break;
        //-----------2Hz signal------------
        case 2:
			if(!(buzzer_d.i%5)) PORTF.OUTTGL = PIN6_bm;
			buzzer_d.i++;
            break;
        //----------cont signal------------
        case 3:
            PORTF.OUTSET = PIN6_bm;
            break;
		//----------Short beep--------------
		case 4:
			PORTF.OUTSET = PIN6_bm;
			buzzer_d.mode = 0;
			break;
		//----------2x beep-----------------
		case 5:
		if(buzzer_d.count > 3) buzzer_d.mode = 0;	//wy��czenie po 2 sygna�ach
		else if(buzzer_d.count <= 3) {
			if(!(buzzer_d.i%3)){
				PORTF.OUTTGL = PIN6_bm;
				buzzer_d.count++;
			}
			buzzer_d.i++;
		}
		break;
        }
    } else {
        buzzer_d.mode = 0;
        PORTF.OUTCLR = PIN6_bm;
    }
}

//----------------------IO update-------------------------------------
ISR(TCE0_OVF_vect) {
    //-------------Blokada komend zdalnych----------------------------
    LED_PORT.OUTCLR = LED3;
    stan_d.cmd_mode = false;
}

//----------------------Frame send------------------------------------
ISR(TCF0_OVF_vect) {
    LED_PORT.OUTTGL = LED4;
    frame_d.terminate = false;
    if(stan_d.telemetry_trigger) {
        if(RTC_d.frameTeleCount< 99999) RTC_d.frameTeleCount++;
        else RTC_d.frameTeleCount = 0;
		//----- Begin transmission -----
        frame_d.iUART = 0;
		//XBEE_UART.DATA = '$';	// alternatywnie 0 lub '\r'
        USARTD0_TXC_vect();
    }
}

//---------------------Buzzer functions-------------------------------
void Buzzer3Beep(void){
	buzzer_d.mode = 1;
	buzzer_d.trigger = true;
}

void Buzzer2Hz(void){
	buzzer_d.mode = 2;
	buzzer_d.trigger = true;
}

void BuzzerContBeep(void){
	buzzer_d.mode = 3;
	buzzer_d.trigger = true;
}

void Buzzer1Beep(void){
	buzzer_d.mode = 4;
	buzzer_d.trigger = true;
}

void Buzzer2Beep(void){
	buzzer_d.mode = 5;
	buzzer_d.trigger = true;
}

//---------------------Parachutes functions---------------------
void Parachute1deploy(){
	
}

void Parachute2deploy(){
	
}

//----------------------Memory erase---------------------------
void FLASHerase(void) {
    const char buf0[] = "\n\rMemory erased!\n\r\n\r\0";
    while((!(PORTE.IN & PIN0_bm)) || (!(PORTE.IN & PIN1_bm))) {}
    buzzer_d.mode = 3;																//sygna� 2Hz
    buzzer_d.trigger = true;														//odblokowanie buzzera
    SPI_ChipErase();
    _delay_ms(1000);
    uint16_t i = 0;
    while(buf0[i]) {
        XBEE_UART.DATA = buf0[i++];
        _delay_ms(2);
    }
    buzzer_d.trigger = false;														//odblokowanie buzzera
    SPIaddress = 0;
}

void FLASH_saveData(allData_t * allData_d){
	FLASH_dataStruct_t FLASH_struct_d;
	FLASH_struct_d.marker = 0xAA;
	/*
	FLASH_struct_d.IGN = allData_d->stan->IGN;
	FLASH_struct_d.MFV = allData_d->stan->MFV;
	FLASH_struct_d.MOV = allData_d->stan->MOV;
	FLASH_struct_d.WPV = allData_d->stan->MPV;
	FLASH_struct_d.FPV = allData_d->stan->FPV;
	
	FLASH_struct_d.press1 = allData_d->AD7195->raw_press1;
	FLASH_struct_d.press2 = allData_d->AD7195->raw_press2;
	FLASH_struct_d.press3 = allData_d->AD7195->raw_press3;
	FLASH_struct_d.press4 = allData_d->AD7195->raw_press4;
	FLASH_struct_d.press5 = allData_d->AD7195->raw_press5;
	FLASH_struct_d.press6 = allData_d->AD7195->raw_press6;
	FLASH_struct_d.press7 = allData_d->AD7195->raw_press7;
	FLASH_struct_d.press8 = allData_d->AD7195->raw_press8;
	
	FLASH_struct_d.temp1 = allData_d->Analog->Temp1;
	FLASH_struct_d.temp2 = allData_d->Analog->Temp2;
	FLASH_struct_d.temp3 = allData_d->Analog->Temp3;
	FLASH_struct_d.temp4 = allData_d->Analog->Temp4;
	
	FLASH_struct_d.Clock = allData_d->Clock->RealTime;
	
	uint8_t pagePosition = allData_d->FLASH_pageStruct->position;
	if (pagePosition < 8){
		allData_d->FLASH_pageStruct->FLASH_dataStruct[pagePosition] = FLASH_struct_d;
		allData_d->FLASH_pageStruct->position++;
	}
	if(pagePosition >=8){
		FLASH_pageWrite(allData_d->FLASH_pageStruct->pageNo, allData_d->FLASH_pageStruct->data, 512);
		allData_d->FLASH_pageStruct->pageNo++;
		allData_d->FLASH_pageStruct->position = 0;
	}
	*/
}

//----------------------Kalibracja wsystkich czujnik�w-----------------
void SensorCal(void) {
    if(Calibration_d.counter < 1) {
        Calibration_d.BMP_pressure = 0;
        Calibration_d.LPS_pressure = 0;
        Calibration_d.MPU_gyro_x = 0;
        Calibration_d.MPU_gyro_y = 0;
        Calibration_d.MPU_gyro_z = 0;
        Calibration_d.LSM_gyro_x = 0;
        Calibration_d.LSM_gyro_y = 0;
        Calibration_d.LSM_gyro_z = 0;
        LED_PORT.OUTSET = LED5;
    }
    if(Calibration_d.counter < calibrationCNT) {
        Calibration_d.LPS_pressure += LPS25H_d.pressure;
        Calibration_d.MPU_gyro_x += MPU9150_d.raw_gyro_x;
        Calibration_d.MPU_gyro_y += MPU9150_d.raw_gyro_y;
        Calibration_d.MPU_gyro_z += MPU9150_d.raw_gyro_z;
        Calibration_d.LSM_gyro_x += LSM9DS0_d.raw_gyro_x;
        Calibration_d.LSM_gyro_y += LSM9DS0_d.raw_gyro_y;
        Calibration_d.LSM_gyro_z += LSM9DS0_d.raw_gyro_z;
        Calibration_d.counter++;
    } else {
        LPS25H_d.start_pressure = Calibration_d.LPS_pressure / calibrationCNT;
        MPU9150_d.offset_gyro_x = Calibration_d.MPU_gyro_x / calibrationCNT;
        MPU9150_d.offset_gyro_y = Calibration_d.MPU_gyro_y / calibrationCNT;
        MPU9150_d.offset_gyro_z = Calibration_d.MPU_gyro_z / calibrationCNT;
        LSM9DS0_d.offset_gyro_x = Calibration_d.LSM_gyro_x / calibrationCNT;
        LSM9DS0_d.offset_gyro_y = Calibration_d.LSM_gyro_y / calibrationCNT;
        LSM9DS0_d.offset_gyro_z = Calibration_d.LSM_gyro_z / calibrationCNT;
        Calibration_d.counter = 0;
        Calibration_d.trigger = false;
        stan_d.callibration = false;
        LED_PORT.OUTCLR = LED5;
    }
}

void structInit(void) {
    frame_d.iUART = 0;
    stan_d.new_data = false;
    stan_d.new_frame = false;
	
	//----------------------Initialize allData_d--------------------
	allData_d.Analog = &ADC_d;
	allData_d.MPU9150 = &MPU9150_d;
	allData_d.LSM9DS0 = &LSM9DS0_d;
	allData_d.LPS25H = &LPS25H_d;
	allData_d.GPS = &GPS_d;
	allData_d.stan = &stan_d;
	allData_d.LIS331HH = &LIS331HH_d;
	allData_d.SensorsData = &SensorData_d;
	allData_d.frame = &frame_d;
	allData_d.frame_b = &frame_b;
	allData_d.boardOrient = &boardOrient_d;
	allData_d.RTC = &RTC_d;
	allData_d.FLASH_pageStruct = &FLASH_pageStruct_d;
}

void BT_Start(frame_t * frame) {
    int i = 0;
    frame->frameASCII[i++] = '\r';	//\r\n+INQ=1\r\n
    frame->frameASCII[i++] = '\n';
    frame->frameASCII[i++] = '+';
    frame->frameASCII[i++] = 'I';
    frame->frameASCII[i++] = 'N';
    frame->frameASCII[i++] = 'Q';
    frame->frameASCII[i++] = '=';
    frame->frameASCII[i++] = '1';
    frame->frameASCII[i++] = '\r';
    frame->frameASCII[i++] = '\n';
	frame->frameASCII[i++] = 0;
	frame->frameASCII[i++] = 0;
    frame->iUART = 0;
	
	USARTD0_TXC_vect();
	_delay_ms(100);
}

void SensorUpdate(allData_t * allData) {
    //-----------------MPU9150--------------
    MPU9150_RawUpdate(allData->MPU9150);
    MPU9150_Conv(allData->MPU9150);		//110us
    //-----------------LIS331HH-------------
    LIS331HH_Update(allData->LIS331HH);
    LIS331HH_Calc(allData->LIS331HH);	//30us
    //-----------------LPS25H---------------
    LPS25H_update(allData->LPS25H);
    LPS25H_calc(allData->LPS25H);		//25us
    altitudeCalcLPS(allData->LPS25H);	//225us
    //-----------------LSM9DS0--------------
    LSM9DS0_Update(allData->LSM9DS0);
    //-----------------Read ADC-------------
    AnalogUpdate(allData->Analog);
	
	
	//do router lub funkcji i wywali� jak najwi�cej!
    //-----------------Additional-----------
    if(LPS25H_d.altitude > LPS25H_d.max_altitude) LPS25H_d.max_altitude = LPS25H_d.altitude;
}

void StateUpdate(allData_t * allData) {
    if(!(stan_d.armed_trigger)) {
        stan_d.flightState = 0;
        buzzer_d.mode = 0;
		stan_d.flash_trigger = false;
        PORTD_OUTCLR = PIN1_bm;
    } else {
        switch(stan_d.flightState) {
        //--------case 0 preflight-------------------------------------------
        case 0:
            if((SensorData_d.accel_x > 3) /*&& (LPS25H_d.velocity > 10)*/){		//wykrycie startu (Arecorder Acc+Alti) (Arecorder zapisuje kilkana�cie pr�bek wstecz)
				stan_d.flash_trigger = true;
				Buzzer2Beep();
				if(dual_stage) stan_d.flightState = 1;	
				else stan_d.flightState = 4;
			}
			if(SensorData_d.accel_x < 2) {
				LPS25H_d.start_pressure = 0.9*LPS25H_d.start_pressure + 0.1*LPS25H_d.pressure;		//uaktualniaj ci�nienie na starcie
				DetectInitOrientationCont(&allData_d);	//detect orientation and angle
			}
			if(SensorData_d.accel_x > 2) Buzzer1Beep();
			LPS25H_d.max_altitude = LPS25H_d.altitude;	//uaktualniaj max wysoko�� na starcie
            break;
			
		//-------case 1 wait for MECO-----------------------------------------
		case 1:
			if((SensorData_d.accel_x < 0) && (SensorData_d.altitude > 50) && (SensorData_d.ascentVelo > 50)){	//Arecorder filtruje przyspieszenie
				timer_buffer = RTC_d.time;														//buforowanie czasu
				stan_d.flightState = 2;
			}
			break;
			
		//-------case 2 separation delay---------------------------------------
		case 2:
			if(RTC_d.time > (timer_buffer + 1000)){
				//StagesSeparation();
				timer_buffer = RTC_d.time;
				stan_d.flightState = 3;
			}
			break;
		
		//-------case 3 SEI delay----------------------------------------------
		case 3:
			if(RTC_d.time > (timer_buffer + 2000)){
				//SecondEngineIgn();
				stan_d.flightState = 4;
			}
			break;
			
        //--------case 4 flight wait for apogee--------------------------------
        case 4:
            if(((LPS25H_d.max_altitude - LPS25H_d.altitude) > 10.0) && (LPS25H_d.velocity < 0)) stan_d.flightState = 5;	//wykrycie pu�apu
            break;
			
        //-------case 5 sound signal + pilot parachute deployment------------------
        case 5:
            Buzzer3Beep();				//3 sygna�y d�wi�kowe
			Parachute1deploy();			//wyrzucenie spadochronu nr 1
            stan_d.flightState = 6;
            break;
			
        //--------case 6 wait for main parachute target or pilot failure-----------
        case 6:
            if(allData->SensorsData->altitude < 400) stan_d.flightState = 7;						//warunek wysoko�ci
			//if(allData_d->SensorsData->accel_press < -1) stan_d.flightState = 7;					//warunek zwi�kszania pr�dko�ci opadania
			if(allData->SensorsData->ascentVelo < -30) stan_d.flightState = 7;					//warunek pr�dko�ci
            break;
			
        //--------case 7 main parachute deployment-----------
        case 7:
			Parachute2deploy();							//wyrzucenie g��wnego spadochronu
			Buzzer2Beep();								//2 sygna�y d�wi�kowe
            stan_d.flightState = 8;	
            break;
			
        //---------case 9 wait for landing----------
        case 8:
            if((LPS25H_d.altitude < 200) && (LPS25H_d.velocity < 1) && (LPS25H_d.velocity > -1)) stan_d.flightState = 9;
            break;
			
        //---------case 9 END----------------
        case 9:
            Buzzer2Hz();
			stan_d.flash_trigger = false;
            break;
        }
    }
}

void Initialization(void) {
	CPU_clk(CPU_clock);	//zegar CPU
    OscRTC();			//zegar RTC
    RTC_Init();			//konfiguracja i uruchomienie RTC
    ADC_Init();			//inicjalizacja ADC
    USART_Init();		//inicjalizacja Xbee
    GPS_Init();			//inicjalizacja GPS
    GPS_Conf();			//wys�anie konfiguracji do GPS
    IO_Init();
    TimerCInit(sampling_time);	//sensor update
    TimerDInit(50);				//buzzer handling
    TimerEInit(50);				//obs�uga IO
    TimerFInit(telemetry_time);	//frame send
    structInit();
    I2C_Init();
    //--------MPU9150 Init-----------
    MPU9150_WakeUp();
    //-------LPS25H Init------------
    LPS25H_config();
    //-------LIS331HH Init---------
    LIS331HH_WakeUp();
    //-------LSM9DS0 Init----------
    LSM9DS0_Init();
    //-------SPI Flash Init--------
    SPI_Init();
    SPI_WriteProtection();
    SPIaddress = SPI_FindEnd(128);		//szukaj wolnego miejsca w pami�ci------------------------------------------------
    //SPIaddress = 0;
    //-------w��czenie przerwa�----
    PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
}

void InitMemoryErase() {
    uint32_t i = 0;
    const char buf0[] = "\n\rMemory erased!\n\r\n\r\0";
    while((!(PORTE.IN & PIN0_bm)) || (!(PORTE.IN & PIN1_bm))) {}
    buzzer_d.mode = 3;																//sygna� 2Hz
    buzzer_d.trigger = true;														//odblokowanie buzzera
    SPI_ChipErase();
    _delay_ms(1000);
    i = 0;
    while(buf0[i]) {
        XBEE_UART.DATA = buf0[i++];
        _delay_ms(2);
    }
    buzzer_d.trigger = false;														//odblokowanie buzzera
    SPIaddress = 0;
}

void InitMemoryRead() {
    uint32_t i = 0;
    stan_d.flash_trigger = false;
    buzzer_d.mode = 3;																//sygna� 2Hz
    buzzer_d.trigger = true;
    _delay_ms(200);
    buzzer_d.trigger = false;
    while(!(PORTE.IN & PIN0_bm)) {}
    const char buf1[] = "\n\rID,TimeMS,FlightState,Bat,Altitude,Velocity,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Lat,Long,AltiGPS\n\r\n\r\0";
    const char buf2[] = "\n\rReading done\n\r\n\r\0";
    //------Hello message----------
    while(buf1[i]) {
        XBEE_UART.DATA = buf1[i++];
        _delay_ms(10);
    }
    _delay_ms(1000);
    //-------start Flash read------S
    uint8_t ch = 0xFF;
    uint8_t FFcnt = 0;
    LED_PORT.OUTSET = LED1;
    SPI_CS(true);
    SPI_W_Byte(0x03);					//Read
    SPI_W_Byte(0);	//address MSB
    SPI_W_Byte(0);	//address cd.
    SPI_W_Byte(0);	//address LSB
    i = 0;
    do {
        ch = SPI_R_Byte();
        if(ch != 0xFF) {
			if(ch == '$') {
				XBEE_UART.DATA = '\n';
				_delay_us(100);
			}
            if(ch != 0) XBEE_UART.DATA = ch;
            FFcnt = 0;
        } else FFcnt++;
        i++;
        if((i % 128) == 0) _delay_ms(5);
        else _delay_us(100);
    } while((PORTE.IN & PIN0_bm) && (FFcnt < 100));
    SPI_CS(false);
	
	//-------Reading done-------------
    i = 0;
    while(buf2[i]) {
        XBEE_UART.DATA = buf2[i++];
        _delay_ms(1);
    }
	
	//-----Hang--------------
	while(1) {}
}

void WarmUp() {
    //------Bluetooth bee init-----
    //_delay_ms(1000);
    BT_Start(&frame_d);
	
    //------Hello blink
    LED_PORT.OUTSET = LED1 | LED2 | LED3 | LED4 | LED5;
    _delay_ms(200);
    LED_PORT.OUTCLR = LED1;
    _delay_ms(200);
    LED_PORT.OUTCLR = LED2;
    _delay_ms(200);
    LED_PORT.OUTCLR = LED3;
    _delay_ms(200);
	LED_PORT.OUTCLR = LED4;
	_delay_ms(200);
	LED_PORT.OUTCLR = LED5;
	_delay_ms(200);
}

void WarmUpMemoryOperations() {
    //----------------Kasowanie pami�ci Flash------------------------
    if((!(PORTE.IN & PIN0_bm)) && (!(PORTE.IN & PIN1_bm))) InitMemoryErase();
    //-----------------Odczyt z pami�ci i wys�anie po Xbee-----------
    else if(!(PORTE.IN & PIN0_bm)) InitMemoryRead();
}

bool DetectInitOrientation(allData_t * allData){
	//napisa� funkcj� wykrywaj�c� orientacj� na starcie
	//----Local variable------------------------------------------------
	volatile float accTotal = 0;
	volatile float mean_accX = 0;
	volatile float mean_accY = 0;
	volatile float mean_accZ = 0;
	volatile float abs_mean_accX = 0;
	volatile float abs_mean_accY = 0;
	volatile float abs_mean_accZ = 0;
	
	//----Attach local pointer to main data struct----------------------
	float * accX = &(allData->SensorsData->accel_x);
	float * accY = &(allData->SensorsData->accel_y);
	float * accZ = &(allData->SensorsData->accel_z);
	
	//-------Update all sensors data and calculate mean from n sample----
	for(uint8_t i=0;i<100;i++){
		SensorUpdate(&allData_d);
		SensorDataFusion(&allData_d);
		mean_accX += *accX;
		mean_accY += *accY;
		mean_accZ += *accZ;
	}
	mean_accX /= 100.0;
	mean_accY /= 100.0;
	mean_accZ /= 100.0;
	accTotal = VectorLength3D(*accX, *accY, *accZ);
	abs_mean_accX = fabs(mean_accX);
	abs_mean_accY = fabs(mean_accY);
	abs_mean_accZ = fabs(mean_accZ);
	
	//-------Check for errors---------------------------------------------	//mo�e warto da� powt�rny pomiar?
	if((abs(accTotal) < 0.9) || (abs(accTotal) > 1.1)) return 1;
	
	//-------Determine main orientation-----------------------------------
	if((abs_mean_accX > abs_mean_accY) && (abs_mean_accX > abs_mean_accZ)) allData->boardOrient->config = 1;		//X axis = main
	else if((abs_mean_accY > abs_mean_accX) && (abs_mean_accY > abs_mean_accZ)) allData->boardOrient->config = 2;	//Y axis = main
	else allData->boardOrient->config = 3;																			//Z axis = main
	
	switch(allData->boardOrient->config){
		case 1: if(mean_accX < 0.1) allData->boardOrient->invert = true; break;
		case 2: if(mean_accY < 0.1) allData->boardOrient->invert = true; break;
		case 3: if(mean_accZ < 0.1) allData->boardOrient->invert = true; break;
	}
	
	//-------Calculate launchpad angle------------------------------------
	MinAngleVector3D(abs_mean_accX, abs_mean_accY, abs_mean_accZ);
	return 0;
}

bool DetectInitOrientationCont(allData_t * allData){
	//napisa� funkcj� wykrywaj�c� orientacj� na starcie
	//----Local variable------------------------------------------------
	volatile float accTotal = 0;
	volatile float abs_mean_accX = 0;
	volatile float abs_mean_accY = 0;
	volatile float abs_mean_accZ = 0;
	
	//----Attach local pointer to main data struct----------------------
	float * accX = &(allData->MPU9150->accel_x);
	float * accY = &(allData->MPU9150->accel_y);
	float * accZ = &(allData->MPU9150->accel_z);
	
	float * mean_accX = &(allData->boardOrient->AccelX);
	float * mean_accY = &(allData->boardOrient->AccelY);
	float * mean_accZ = &(allData->boardOrient->AccelZ);
	
	//------- Exp filter ------------------------------
	*mean_accX = 0.9*(*mean_accX) + 0.1*(*accX);
	*mean_accY = 0.9*(*mean_accY) + 0.1*(*accY);
	*mean_accZ = 0.9*(*mean_accZ) + 0.1*(*accZ);
	
	accTotal = VectorLength3D(*accX, *accY, *accZ);
	abs_mean_accX = fabs((*mean_accX));
	abs_mean_accY = fabs((*mean_accY));
	abs_mean_accZ = fabs((*mean_accZ));
	
	//-------Check for errors---------------------------------------------
	if((abs(accTotal) < 0.9) || (abs(accTotal) > 1.1)) return 1;
	
	//-------Determine main orientation-----------------------------------
	if((abs_mean_accX > abs_mean_accY) && (abs_mean_accX > abs_mean_accZ)) allData->boardOrient->config = 1;		//X axis = main
	else if((abs_mean_accY > abs_mean_accX) && (abs_mean_accY > abs_mean_accZ)) allData->boardOrient->config = 2;	//Y axis = main
	else allData->boardOrient->config = 3;																			//Z axis = main
	
	switch(allData->boardOrient->config){
		case 1: if((*mean_accX) < 0.1) allData->boardOrient->invert = true; else allData->boardOrient->invert = false; break;
		case 2: if((*mean_accY) < 0.1) allData->boardOrient->invert = true; else allData->boardOrient->invert = false; break;
		case 3: if((*mean_accZ) < 0.1) allData->boardOrient->invert = true; else allData->boardOrient->invert = false; break;
	}
	
	//-------Calculate launchpad angle------------------------------------
	allData->boardOrient->angle = MinAngleVector3D(abs_mean_accX, abs_mean_accY, abs_mean_accZ);
	return 0;
}

void SensorDataFusion(allData_t * allData){
	//fuzja danych z czujnik�w - na pocz�tek tylko akcelerometry
	//--------Pointer to correct data------------------------------
	float MPU9150_accel_x;
	float MPU9150_accel_y;
	float MPU9150_accel_z;
	float MPU9150_gyro_x;
	float MPU9150_gyro_y;
	float MPU9150_gyro_z;
	float MPU9150_mag_x;
	float MPU9150_mag_y;
	float MPU9150_mag_z;
	float LSM9DS0_accel_x;
	float LSM9DS0_accel_y;
	float LSM9DS0_accel_z;
	float LSM9DS0_gyro_x;
	float LSM9DS0_gyro_y;
	float LSM9DS0_gyro_z;
	float LSM9DS0_mag_x;
	float LSM9DS0_mag_y;
	float LSM9DS0_mag_z;
	float LIS331HH_accel_x;
	float LIS331HH_accel_y;
	float LIS331HH_accel_z;
	
	//--------Sensor data router------------------------------------
	switch(allData->boardOrient->config){
		case 1:	//-----X sensor = main axis
		default:
		MPU9150_accel_x = allData->MPU9150->accel_x;
		MPU9150_accel_y = allData->MPU9150->accel_y;
		MPU9150_accel_z = allData->MPU9150->accel_z;
		MPU9150_gyro_x = allData->MPU9150->gyro_x;
		MPU9150_gyro_y = allData->MPU9150->gyro_y;
		MPU9150_gyro_z = allData->MPU9150->gyro_z;
		MPU9150_mag_x = allData->MPU9150->mag_x;
		MPU9150_mag_y = allData->MPU9150->mag_y;
		MPU9150_mag_z = allData->MPU9150->mag_z;
		LSM9DS0_accel_x = allData->LSM9DS0->accel_x;
		LSM9DS0_accel_y = allData->LSM9DS0->accel_y;
		LSM9DS0_accel_z = allData->LSM9DS0->accel_z;
		LSM9DS0_gyro_x = allData->LSM9DS0->gyro_x;
		LSM9DS0_gyro_y = allData->LSM9DS0->gyro_y;
		LSM9DS0_gyro_z = allData->LSM9DS0->gyro_z;
		LSM9DS0_mag_x = allData->LSM9DS0->mag_x;
		LSM9DS0_mag_y = allData->LSM9DS0->mag_y;
		LSM9DS0_mag_z = allData->LSM9DS0->mag_z;
		LIS331HH_accel_x = allData->LIS331HH->accel_x;
		LIS331HH_accel_y = allData->LIS331HH->accel_y;
		LIS331HH_accel_z = allData->LIS331HH->accel_z;
		break;
		case 2: //-----Y sensor = main axis
		MPU9150_accel_x = allData->MPU9150->accel_y;
		MPU9150_accel_y = allData->MPU9150->accel_x;
		MPU9150_accel_z = allData->MPU9150->accel_z;
		MPU9150_gyro_x = allData->MPU9150->gyro_y;
		MPU9150_gyro_y = allData->MPU9150->gyro_x;
		MPU9150_gyro_z = allData->MPU9150->gyro_z;
		MPU9150_mag_x = allData->MPU9150->mag_y;
		MPU9150_mag_y = allData->MPU9150->mag_x;
		MPU9150_mag_z = allData->MPU9150->mag_z;
		LSM9DS0_accel_x = allData->LSM9DS0->accel_y;
		LSM9DS0_accel_y = allData->LSM9DS0->accel_x;
		LSM9DS0_accel_z = allData->LSM9DS0->accel_z;
		LSM9DS0_gyro_x = allData->LSM9DS0->gyro_y;
		LSM9DS0_gyro_y = allData->LSM9DS0->gyro_x;
		LSM9DS0_gyro_z = allData->LSM9DS0->gyro_z;
		LSM9DS0_mag_x = allData->LSM9DS0->mag_y;
		LSM9DS0_mag_y = allData->LSM9DS0->mag_x;
		LSM9DS0_mag_z = allData->LSM9DS0->mag_z;
		LIS331HH_accel_x = allData->LIS331HH->accel_y;
		LIS331HH_accel_y = allData->LIS331HH->accel_x;
		LIS331HH_accel_z = allData->LIS331HH->accel_z;
		break;
		case 3: //-----Z sensor = main axis
		MPU9150_accel_x = allData->MPU9150->accel_z;
		MPU9150_accel_y = allData->MPU9150->accel_y;
		MPU9150_accel_z = allData->MPU9150->accel_x;
		MPU9150_gyro_x = allData->MPU9150->gyro_z;
		MPU9150_gyro_y = allData->MPU9150->gyro_y;
		MPU9150_gyro_z = allData->MPU9150->gyro_x;
		MPU9150_mag_x = allData->MPU9150->mag_z;
		MPU9150_mag_y = allData->MPU9150->mag_y;
		MPU9150_mag_z = allData->MPU9150->mag_x;
		LSM9DS0_accel_x = allData->LSM9DS0->accel_z;
		LSM9DS0_accel_y = allData->LSM9DS0->accel_y;
		LSM9DS0_accel_z = allData->LSM9DS0->accel_x;
		LSM9DS0_gyro_x = allData->LSM9DS0->gyro_z;
		LSM9DS0_gyro_y = allData->LSM9DS0->gyro_y;
		LSM9DS0_gyro_z = allData->LSM9DS0->gyro_x;
		LSM9DS0_mag_x = allData->LSM9DS0->mag_z;
		LSM9DS0_mag_y = allData->LSM9DS0->mag_y;
		LSM9DS0_mag_z = allData->LSM9DS0->mag_x;
		LIS331HH_accel_x = allData->LIS331HH->accel_z;
		LIS331HH_accel_y = allData->LIS331HH->accel_y;
		LIS331HH_accel_z = allData->LIS331HH->accel_x;
		break;
	}
	// Rotate vector if up side down
	if(allData->boardOrient->invert){
		MPU9150_accel_x = -MPU9150_accel_x;
		MPU9150_gyro_x = -MPU9150_gyro_x;
	}
	//-----Sensor fusion---------------------------------------
	// Tu b�d� dzia� si� czary
	// Tymczsowo zwyk�e przepisanie zmiennych
	// Doda� odejmowanie grawitacji
	allData->SensorsData->accel_x = MPU9150_accel_x;
	allData->SensorsData->accel_y = MPU9150_accel_y;
	allData->SensorsData->accel_z = MPU9150_accel_z;
	allData->SensorsData->gyro_x = MPU9150_gyro_x;
	allData->SensorsData->gyro_y = MPU9150_gyro_y;
	allData->SensorsData->gyro_z = MPU9150_gyro_z;
	allData->SensorsData->mag_x = LSM9DS0_mag_x;
	allData->SensorsData->mag_y = LSM9DS0_mag_y;
	allData->SensorsData->mag_z = LSM9DS0_mag_z;
	allData->SensorsData->altitude = allData->LPS25H->altitude;
	allData->SensorsData->ascentVelo = allData->LPS25H->velocity;
	
	//------ Orientation update -------------------------------
	OrientationUpdate(allData);
	
	//------ Real Accelerations calc --------------------------
	AccelerationCorrection(allData);
	
	//------ Velocity calc ------------------------------------
	VelocityUpdate(allData);
	
	//------ Position update ----------------------------------
	PositionUpdate(allData);
}

void PositionUpdate(allData_t * allData){
	
}

void AccelerationCorrection(allData_t * allData){
	
}

void VelocityUpdate(allData_t * allData){
	
}

void OrientationUpdate(allData_t * allData){
	
}

void CalibrationStart() {
    Calibration_d.counter = 0;
    Calibration_d.trigger = true;		//zdalna kalibracja czujnik�w
    stan_d.callibration = true;
}

int main(void) {
    //_delay_ms(10);	//160ms przy 2MHZ na starcie
    //sprawdznie poprawno�ci danych w strukturze
    frame_d.terminate = false;
    Initialization();
	DetectInitOrientation(&allData_d);	//detect orientation and angle
	WarmUp();					//odmiganie startu
    sei();
	BT_Start(&frame_d);			//inicjalizacja BT
    WarmUpMemoryOperations();	//odczyt lub kasowanie pami�ci
    CalibrationStart();			//autocalibration
	stan_d.flash_trigger = STARTUP_flash;
	stan_d.telemetry_trigger = STARTUP_tele;
	stan_d.armed_trigger = STARTUP_armed;
	
    while(1) {
        _delay_us(1);
        if(stan_d.new_data == true) {
			if(stan_d.armed_trigger) LED_PORT.OUTTGL = LED1;
			else LED_PORT.OUTCLR = LED1;
            //============================================================================
            //								Sensors update
            //============================================================================
            stan_d.new_data = false;	//DRY flag clear
            SensorUpdate(&allData_d);		//2320us
			SensorDataFusion(&allData_d);	//8us
            StateUpdate(&allData_d);		//21us
			
            //----------------Prepare frame---------
            prepareFrame(&allData_d);	//333us
            if(stan_d.flash_trigger){
				SPI_WriteFrame(&SPIaddress, 128, (uint8_t *)(frame_b.frameASCII));
				if(RTC_d.frameFlashCount < 999999UL) RTC_d.frameFlashCount++;
				else RTC_d.frameFlashCount = 0;
			}
            if(!(frame_d.mutex)) frame_d = frame_b;							//je�li frame_d nie zablokowane -> przepisz z bufora
            
            //----------------Kalibracja------------
            if(Calibration_d.trigger) SensorCal();
        }
    }
}