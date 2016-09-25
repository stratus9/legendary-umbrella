/*
 * MPU9150.h
 *
 * Created: 2015-03-22 19:10:52
 *  Author: stratus
 */ 


#ifndef MPU9150_H_
#define MPU9150_H_

void MPU9150_WakeUp(void);
void MPU9150_RawUpdate(MPU9150_t * data);
void MPU9150_Calc(MPU9150_t * dane);
void MPU9150_RegWrite(uint8_t reg, uint8_t value);
void MPU9150_MagRegWrite(uint8_t reg, uint8_t value);
void MPU9150_MagInit(void);
void MPU9150_MagUpdate(MPU9150_t * data);
void MPU9150_MagCalc(MPU9150_t * data);
void MPU9150_MagCal(MPU9150_t * data);
uint8_t MPU9150_WhoAmI(void);
void MPU9150_Conv(allData_t *);
void MPU9150_MagStartConv(void);
#endif /* MPU9150_H_ */
