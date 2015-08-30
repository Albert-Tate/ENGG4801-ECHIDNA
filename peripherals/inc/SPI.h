 /**********************************************************
 *  File: SPI.h
 *  Description: Driver for SPI
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
 
#ifndef __SPI_H__
#define __SPI_H__
 
 void SPI_setup(void);
 uint8_t SPI_transmit_byte(uint8_t byte);
 
 
 #endif