/*
 * util.h
 *
 * Created: 2015-03-22 20:24:23
 *  Author: stratus
 */ 

#ifndef UTIL_H_
#define UTIL_H_

void float2char(float number,char * tablica);
bool purgeBuffer(struct ringBuffer_t * bufor);
bool ringBuffer_addString(struct ringBuffer_t * bufor, char * text, uint16_t text_length);
bool ringBuffer_addChar(struct ringBuffer_t * bufor, char value);
void prepareFrame(struct frame_t *  frame, struct stan_t *  stan, struct GPS_t * gps);
bool GPSdecode(struct ringBuffer_t * bufor, struct GPS_t * gps);
int NMEAchecksum(char *s);
char ringBuffer_read(struct ringBuffer_t * bufor);
void decodeNMEA(struct GPS_t * GPS, struct ringBuffer_t * GPSbuf);
void decodeTime(struct GPS_t GPS);
void altitudeCalc(struct BMP085_t * BMP085, struct frame_t * frame);
void altitudeCalcLPS(struct LPS25H_t * LPS25H, struct frame_t * frame);
void GPS2Frame(struct GPS_t * GPS, struct frame_t * frame);
void GPSbuf_init(struct GPS_t * gps);
void maxAcc(struct frame_t * dane);
#endif /* UTIL_H_ */
