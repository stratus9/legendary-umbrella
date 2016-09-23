/*
 * util.h
 *
 * Created: 2015-03-22 20:24:23
 *  Author: stratus
 */ 

#ifndef UTIL_H_
#define UTIL_H_

void float2char(float number,char * tablica);
bool purgeBuffer(ringBuffer_t * bufor);
bool ringBuffer_addString(ringBuffer_t * bufor, char * text, uint16_t text_length);
bool ringBuffer_addChar(ringBuffer_t * bufor, char value);
void prepareFrame(frame_t *  frame, stan_t *  stan, GPS_t * gps);
bool GPSdecode(ringBuffer_t * bufor, GPS_t * gps);
int NMEAchecksum(char *s);
char ringBuffer_read(ringBuffer_t * bufor);
void decodeNMEA(GPS_t * GPS, ringBuffer_t * GPSbuf);
void decodeTime(GPS_t GPS);
void altitudeCalc(BMP085_t * BMP085, frame_t * frame);
void altitudeCalcLPS(LPS25H_t * LPS25H, frame_t * frame);
void GPS2Frame(GPS_t * GPS, frame_t * frame);
void GPSbuf_init(GPS_t * gps);
void maxAcc(frame_t * dane);
#endif /* UTIL_H_ */
