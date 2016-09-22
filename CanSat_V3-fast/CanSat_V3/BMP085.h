/*
 * BMP085.h
 *
 * Created: 2015-04-17 17:03:05
 *  Author: stratus
 */ 


#ifndef BMP085_H_
#define BMP085_H_

void BMP085_CalUpdate(volatile struct BMP085_t * BMP085);
void BMP085_StartTempConv(void);
void BMP085_StartPressConv(void);
void BMP085_PressUpdate(volatile struct BMP085_t * BMP085);
void BMP085_TempUpdate(volatile struct BMP085_t * BMP085);
void BMP085_Conv(struct BMP085_t * BMP085, struct frame_t * frame);
void BMP085_FastCal(volatile struct BMP085_t * BMP085);
#endif /* BMP085_H_ */
