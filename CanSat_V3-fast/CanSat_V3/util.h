/*
 * util.h
 *
 * Created: 2015-03-22 20:24:23
 *  Author: stratus
 */ 

#ifndef UTIL_H_
#define UTIL_H_
#define VectorLength3D(val1, val2, val3) sqrt(val1*val1 + val2*val2 + val3*val3)
#define VectorLength2D(val1, val2) sqrt(val1*val1 + val2*val2)
#define altitudeCalc(pressure, start_pressure) (1.0-pow(pressure/start_pressure, 0.1902632365))*43538.0

void float2char(float, char *);
bool purgeBuffer(ringBuffer_t * bufor);
bool ringBuffer_addString(ringBuffer_t * bufor, char * text, uint16_t text_length);
bool ringBuffer_addChar(ringBuffer_t * bufor, char value);
void prepareFrame(allData_t *);
bool GPSdecode(ringBuffer_t * bufor, GPS_t * gps);
int NMEAchecksum(char *s);
char ringBuffer_read(ringBuffer_t * bufor);
void decodeNMEA(GPS_t * GPS, ringBuffer_t * GPSbuf);
void decodeTime(GPS_t GPS);
void GPS2Frame(GPS_t * GPS, frame_t * frame);
void GPSbuf_init(GPS_t * gps);
float MinAngleVector3D(float, float, float);
#endif /* UTIL_H_ */
