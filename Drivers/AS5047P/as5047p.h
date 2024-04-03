#ifndef _AS5047P_H
#define _AS5047P_H


#define _AS5047P_H
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0x3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF

#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

uint16_t Parity_bit_Calculate(uint16_t data_2_cal);
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata);
uint16_t AS5047_read(uint16_t add);

#endif 