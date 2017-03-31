/*
 * LIS331HH.h
 *
 * Created: 2015-04-19 10:26:02
 *  Author: stratus
 */ 


#ifndef LIS331HH_H_
#define LIS331HH_H_
void LIS331HH_RegWrite(uint8_t, uint8_t);
void LIS331HH_WakeUp(void);
void LIS331HH_Update(LIS331HH_t *);
void LIS331HH_Calc(LIS331HH_t *);
#endif /* LIS331HH_H_ */
