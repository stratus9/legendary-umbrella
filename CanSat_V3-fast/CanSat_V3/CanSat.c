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
static uint32_t SPIaddress = 0;
static float timer_buffer = 0;
uint32_t mission_time = 0;
uint32_t frame_count = 0;

//----------------------Bad ISR handling------------------------
ISR(BADISR_vect) {
    LED_PORT.OUTTGL = LED2;
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
        //buforowanie fragmentów ramki
        if(tmp == ',') {
            GPS_b.buffer[GPS_b.count++] = ',';
            GPS_b.count = 0;
            GPS_b.data_count++;
            GPS_b.new_data = true;
        } else GPS_b.buffer[GPS_b.count++] = tmp;
        //dekodowanie kolejnych pól
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
    if((frame_d.frameASCII[frame_d.iUART]) && (frame_d.frameASCII[frame_d.iUART] != '#')) {
        frame_d.mutex = true;
        if(frame_d.frameASCII[frame_d.iUART] == '%') XBEE_UART.DATA = '\n';
        else XBEE_UART.DATA = frame_d.frameASCII[frame_d.iUART];
        if(frame_d.iUART < 151) frame_d.iUART++;
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
        Calibration_d.trigger = true;		//zdalna kalibracja czujników
        stan_d.callibration = true;
        stan_d.cmd_mode = false;
    } else if((tmp == 'P') && stan_d.cmd_mode) {
        SPI_ChipErase();					//zerowanie pamiêci FLASH
        SPIaddress = 0;
        stan_d.cmd_mode = false;
        buzzer_d.mode = 1;								//3 sygna³y ci¹g³e
        buzzer_d.trigger = true;						//odblokowanie buzzera
		_delay_ms(1000);
    } else if((tmp == 'X') && stan_d.cmd_mode) {
        stan_d.flash_trigger = true;					//start zapisu do pamiêci FLASH
        stan_d.cmd_mode = false;
    } else if((tmp == 'K') && stan_d.cmd_mode) {
        stan_d.flash_trigger = false;					//koniec zapisu do pamiêci FLASH
        stan_d.cmd_mode = false;
    } else if((tmp == 'S') && stan_d.cmd_mode) {
        stan_d.telemetry_trigger = true;			//rozpoczêcie telemetrii
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
    stan_d.new_data = true;
    frame_b.sec++;
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
        case 1:
            if(buzzer_d.count > 6) buzzer_d.mode = 0;	//wy³¹czenie po 2 sygna³ach
            else if(buzzer_d.count <= 6) {
                PORTF.OUTTGL = PIN6_bm;
                buzzer_d.count++;
            }
            break;
        //-----------2Hz signal------------
        case 2:
            PORTF.OUTTGL = PIN6_bm;
            break;
        //----------cont signal------------
        case 3:
            PORTF.OUTSET = PIN6_bm;
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

//----------------------Frame send-------------------------------------
ISR(TCF0_OVF_vect) {
    LED_PORT.OUTTGL = LED4;
    frame_d.terminate = false;
    mission_time = frame_b.sec;
    if(stan_d.telemetry_trigger) {
        if(frame_b.r_count < 9999) frame_b.r_count++;
        else frame_b.r_count = 0;
        frame_count = frame_b.r_count;
        frame_b.iUART = 0;
        USARTD0_TXC_vect();
    }
}

//----------------------Memory erase---------------------------
void FLASHerase(void) {
    const char buf0[] = "\n\rMemory erased!\n\r\n\r\0";
    while((!(PORTE.IN & PIN0_bm)) || (!(PORTE.IN & PIN1_bm))) {}
    buzzer_d.mode = 3;																//sygna³ 2Hz
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

//----------------------Kalibracja wsystkich czujników-----------------
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
    MPU9150_d.offset_gyro_x = -2318;
    MPU9150_d.offset_gyro_y = 1989;
    MPU9150_d.offset_gyro_z = -591;
    MPU9150_d.offset_accel_x = 0;
    MPU9150_d.offset_accel_y = 0;
    MPU9150_d.offset_accel_z = 0;
    LPS25H_d.start_pressure = 1008;
    frame_d.iUART = 0;
    frame_d.r_count = 0;
    stan_d.new_data = false;
    stan_d.new_frame = false;
	ADC_Read(&ADC_d);
	frame_b.r_voltage = ADC_d.Vsense;
	frame_b.vcc = ADC_d.VCC;
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
    frame->iUART = 0;
}

void SensorUpdate(void) {
    //-----------------MPU9150--------------
    MPU9150_RawUpdate(&MPU9150_d);
    MPU9150_Conv(&MPU9150_d, &frame_b);
    //-----------------LIS331HH-------------
    LIS331HH_Update(&LIS331HH_d);
    LIS331HH_Calc(&LIS331HH_d, &frame_b);
    //-----------------LPS25H---------------
    LPS25H_update(&LPS25H_d);
    LPS25H_calc(&LPS25H_d, &frame_b);
    altitudeCalcLPS(&LPS25H_d, &frame_b);
    //-----------------LSM9DS0--------------
    LSM9DS0_Update(&LSM9DS0_d);
    //-----------------Read ADC-------------
    ADC_Read(&ADC_d);
    frame_b.r_voltage = frame_b.r_voltage*(1.0-BAT_voltage_alpha) + ADC_d.Vsense*BAT_voltage_alpha;
	frame_b.vcc = frame_b.vcc*(1.0-BAT_voltage_alpha) + ADC_d.VCC*BAT_voltage_alpha;
    //-----------------Additional-----------
    if(frame_b.LPS25H_altitude > frame_b.max_altitude) frame_b.max_altitude = frame_b.LPS25H_altitude;
    frame_b.LSM9DS0_accel_x = LSM9DS0_d.accel_x;
    frame_b.LSM9DS0_accel_y = LSM9DS0_d.accel_y;
    frame_b.LSM9DS0_accel_z = LSM9DS0_d.accel_z;
    frame_b.LSM9DS0_gyro_x = LSM9DS0_d.gyro_x;
    frame_b.LSM9DS0_gyro_y = LSM9DS0_d.gyro_y;
    frame_b.LSM9DS0_gyro_z = LSM9DS0_d.gyro_z;
    frame_b.LSM9DS0_mag_x = LSM9DS0_d.mag_x;
    frame_b.LSM9DS0_mag_y = LSM9DS0_d.mag_y;
    frame_b.LSM9DS0_mag_z = LSM9DS0_d.mag_z;
    frame_b.LSM9DS0_temp = LSM9DS0_d.temp;
    frame_b.light1 = ((ADC_d.LS1 * 100) / 255);
    frame_b.light2 = ((ADC_d.LS2 * 100) / 255);
    frame_b.light3 = ((ADC_d.LS3 * 100) / 255);
	maxAcc(&frame_b);
}

void StateUpdate(void) {
    if(!(stan_d.armed_trigger)) {
        frame_b.r_FSWstate = 0;
        buzzer_d.mode = 0;
		//stan_d.flash_trigger = false;
        PORTD_OUTCLR = PIN1_bm;
    } else {
        switch(frame_b.r_FSWstate) {
        //--------case 0 preflight-----------------
        case 0:
            if((frame_b.max_acc > 3) || (frame_b.LPS25H_velocity > 10)) frame_b.r_FSWstate = 1;	//wykrycie startu
            buzzer_d.mode = 0;
			frame_b.max_altitude = frame_b.LPS25H_altitude;
            break;
        //--------case 1 flight wait for apogee----
        case 1:
			stan_d.flash_trigger = true;
            if((frame_b.max_altitude - frame_b.LPS25H_altitude) > 10.0) frame_b.r_FSWstate = 2;	//wykrycie pu³apu
            break;
        //-------case 2 delay + sound signal + deployment------------------
        case 2:
            buzzer_d.mode = 1;																//3 sygna³y ci¹g³e
            buzzer_d.trigger = true;														//odblokowanie buzzera
            timer_buffer = frame_b.sec;														//buforowanie czasu
            frame_b.r_FSWstate = 3;
            break;
        //--------case 3 parachute delay--------------
        case 3:
            if(frame_b.sec > (timer_buffer + 5)) frame_b.r_FSWstate = 4;						//odczekanie po separacji
            break;
        //--------case 4 separation wait-----------
        case 4:
            if(((frame_b.LPS25H_velocity) > 40) || (frame_b.LPS25H_altitude < 250)) frame_b.r_FSWstate = 5;						//odczekanie do separacji
            break;
        //-------case 5 separation------------------
        case 5:
            buzzer_d.mode = 1;																//3 sygna³y ci¹g³e
            buzzer_d.trigger = true;														//odblokowanie buzzera
            timer_buffer = frame_b.sec;														//buforowanie czasu
            frame_b.r_FSWstate = 6;
            break;
        //-------case 6 after separation delay------
        case 6:
            if(frame_b.sec > (timer_buffer + 10)) frame_b.r_FSWstate = 7;						//odczekanie po separacji
            break;
        //---------case 7 wait for landing----------
        case 7:
            if((frame_b.LPS25H_altitude < 100) && (frame_b.LPS25H_velocity < 1) && (frame_b.LPS25H_velocity > -1)) frame_b.r_FSWstate = 8;
            break;
        //---------case 8 END----------------
        case 8:
            buzzer_d.mode = 2;																//sygna³ 2Hz
            buzzer_d.trigger = true;														//odblokowanie buzzera
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
    GPS_Conf();			//wys³anie konfiguracji do GPS
    IO_Init();
    TimerCInit(sampling_time);	//sensor update
    TimerDInit(250);			//buzzer handling
    TimerEInit(50);				//obs³uga IO
    TimerFInit(telemetry_time);	//frame send
    structInit();
    I2C_Init();
    //--------MPU9150 Init-----------
    MPU9150_WakeUp();
    //-------LPS25H Init------------
    LPS25H_config();
	frame_b.max_altitude = 0;
    //-------LIS331HH Init---------
    LIS331HH_WakeUp();
    //-------LSM9DS0 Init----------
    LSM9DS0_Init();
    //-------SPI Flash Init--------
    SPI_Init();
    SPI_WriteProtection(false);
    SPIaddress = SPI_FindEnd();		//szukaj wolnego miejsca w pamiêci------------------------------------------------
    //SPIaddress = 0;
    //-------w³¹czenie przerwañ----
    PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
}

void InitMemoryErase() {
    uint32_t i = 0;
    const char buf0[] = "\n\rMemory erased!\n\r\n\r\0";
    while((!(PORTE.IN & PIN0_bm)) || (!(PORTE.IN & PIN1_bm))) {}
    buzzer_d.mode = 3;																//sygna³ 2Hz
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
    buzzer_d.mode = 3;																//sygna³ 2Hz
    buzzer_d.trigger = true;
    _delay_ms(200);
    buzzer_d.trigger = false;
    while(!(PORTE.IN & PIN1_bm)) {}
    const char buf1[] = "\n\rTeam,TeleCnt,FlightState,SoftState,Altitude,Velocity,Accel,Gyro,Lat,Long,AltiGPS,Fix,Check,,Cnt,VoltageBat,VoltageVCC,Temp,Press,AccY,AccY2,GyroX,GyroZ,GyroY,\n\r\n\r\0";
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
            XBEE_UART.DATA = ch;
            FFcnt = 0;
        } else FFcnt++;
        i++;
        if((i % 100) == 0) _delay_ms(10);
        else _delay_us(100);
    } while((PORTE.IN & PIN0_bm) && (FFcnt < 100));
    SPI_CS(false);
    i = 0;
    while(buf2[i]) {
        XBEE_UART.DATA = buf2[i++];
        _delay_ms(1);
    }
}

void WarmUp() {
    //------Bluetooth bee init-----
    //_delay_ms(1000);
    BT_Start(&frame_d);
    USARTD0_TXC_vect();
    //------Hello blink
    LED_PORT.OUTSET = LED2;
    _delay_ms(200);
    LED_PORT.OUTCLR = LED2;
    _delay_ms(100);
    LED_PORT.OUTSET = LED2;
    _delay_ms(200);
    LED_PORT.OUTCLR = LED2;
    _delay_ms(100);
}

void WarmUpMemoryOperations() {
    //----------------Kasowanie pamiêci Flash------------------------
    if((!(PORTE.IN & PIN0_bm)) && (!(PORTE.IN & PIN1_bm))) InitMemoryErase();
    //-----------------Odczyt z pamiêci i wys³anie po Xbee-----------
    else if(!(PORTE.IN & PIN1_bm)) InitMemoryRead();
}

void DetectInitOrientation(void){
	//napisaæ funkcjê wykrywaj¹c¹ orientacjê na starcie
}

void SensorDataFusion(){
	//fuzja danych z czujników
}

void CalibrationStart() {
    Calibration_d.counter = 0;
    Calibration_d.trigger = true;		//zdalna kalibracja czujników
    stan_d.callibration = true;
}

int main(void) {
    //_delay_ms(10);	//160ms przy 2MHZ na starcie
    //sprawdznie poprawnoðci danych w strukturze
    stan_d.flash_trigger = STARTUP_flash;
    stan_d.telemetry_trigger = STARTUP_tele;
    frame_d.terminate = false;
    Initialization();
    sei();
    WarmUp();					//inicjalizacja BT i odmiganie startu
    WarmUpMemoryOperations();	//odczyt lub kasowanie pamiêci
    CalibrationStart();			//autocalibration
	
    while(1) {
        _delay_us(1);
        if(stan_d.new_data == true) {
            //============================================================================
            //								Sensors update
            //============================================================================
            stan_d.new_data = false;	//DRY flag clear
            SensorUpdate();
            StateUpdate();
            //----------------Prepare frame---------
            prepareFrame(&frame_b, &stan_d, &GPS_d);
            if(stan_d.flash_trigger) SPI_WriteFrame(&SPIaddress, 400, &frame_b);
            if(!(frame_d.mutex)) frame_d = frame_b;							//jeœli frame_d nie zablokowane -> przepisz z bufora
            LED_PORT.OUTTGL = LED1;
            //----------------Kalibracja------------
            if(Calibration_d.trigger) SensorCal();
        }
    }
}