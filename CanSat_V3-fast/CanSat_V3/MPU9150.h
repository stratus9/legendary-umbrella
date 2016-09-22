/*
 * MPU9150.h
 *
 * Created: 2015-03-22 19:10:52
 *  Author: stratus
 */ 


#ifndef MPU9150_H_
#define MPU9150_H_

void MPU9150_WakeUp(void);
void MPU9150_RawUpdate(struct MPU9150_t * data);
void MPU9150_Calc(struct MPU9150_t * dane);
void MPU9150_RegWrite(uint8_t reg, uint8_t value);
void MPU9150_MagRegWrite(uint8_t reg, uint8_t value);
void MPU9150_MagInit(void);
void MPU9150_MagUpdate(struct MPU9150_t * data);
void MPU9150_MagCalc(struct MPU9150_t * data);
void MPU9150_MagCal(struct MPU9150_t * data);
uint8_t MPU9150_WhoAmI(void);
void MPU9150_Conv(struct MPU9150_t * MPU9150,struct frame_t * frame);
void MPU9150_MagStartConv(void);
#endif /* MPU9150_H_ */
