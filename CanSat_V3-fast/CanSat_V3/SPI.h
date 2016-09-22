/*
 * SPI.h
 *
 * Created: 2015-05-15 11:45:41
 *  Author: stratus
 */ 


#ifndef SPI_H_
#define SPI_H_
void SPI_W_Byte(uint8_t byte);
uint8_t SPI_R_Byte(void);
void SPI_CS(bool enable);
bool SPI_MemoryCheck(void);
void SPI_WriteEnable(void);
void SPI_WriteDisable(void);
uint8_t SPI_Status(void);
void SPI_ChipErase(void);
char SPI_Read(uint32_t address,uint16_t size, char * tablica);
void SPI_WriteByte(uint32_t address, uint8_t data);
void SPI_WriteProtection(bool block);
void SPI_CmdSend(char cmd);
void SPI_AAI_Mode_Start(void);
void SPI_AAI_Mode_Stop(void);
void SPI_WriteFin(void);
char SPI_ReadByte(uint32_t address);
uint32_t SPI_FindEnd(void);
void SPI_WriteFrame(uint32_t * adres, uint16_t frame_length, struct frame_t * frame);
//void SPI_PageWrite(uint16_t page, uint16_t page_length, struct frame_t * frame);
#endif /* SPI_H_ */
