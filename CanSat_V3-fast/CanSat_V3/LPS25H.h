/*
 * LPS25H.h
 *
 * Created: 2015-04-18 09:27:30
 *  Author: stratus
 */ 


#ifndef LPS25H_H_
#define LPS25H_H_
void LPS25H_config(void);

void LPS25H_update(LPS25H_t * LPS25H);
void LPS25H_startConv(void);
uint8_t LPS25H_WhoIAm(void);
uint8_t LPS25H_ReadReg(uint8_t adres);
void altitudeCalcLPS(LPS25H_t * LPS25H);
void LPS25H_calc(LPS25H_t * LPS25H);
#endif /* LPS25H_H_ */
