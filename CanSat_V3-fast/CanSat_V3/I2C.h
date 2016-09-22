/*
 * I2C.h
 *
 * Created: 2015-10-25 10:44:20
 *  Author: baretk
 */ 


#ifndef I2C_H_
#define I2C_H_
uint8_t I2C_ReadEnd(bool);
void I2C_WriteReg(uint8_t I2Caddres, uint8_t reg, uint8_t data);
void I2C_ReadRegister(uint8_t I2Caddres, uint8_t reg, uint8_t length, uint8_t * data);
#endif /* I2C_H_ */