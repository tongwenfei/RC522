#ifndef __AD7705_H
#define __AD7705_H 			   
#include <sys.h>
#include "delay.h"
#define AD7705_SCLK PHout(13)
#define AD7705_MOSI PCout(1)
#define AD7705_MISO PCin(3)
#define AD7705_RST  PHout(15)
#define AD7705_DRDY PCin(5)
#define UA0 PCout(2)
#define UA1 PCout(4)
#define UA2 PBout(0)
#define UEA PCout(0)

void AD7705_Reset(void) ;
uint8_t SPI_ReadByte(void);
uint16_t SPI_ReadHalfWord(void);
void SPI_WriteByte(uint8_t Data);
void Get_Voltage(void);
#endif