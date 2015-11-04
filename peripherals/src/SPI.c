/*
NVIS Inc.
Jianyi Liu
Created		Jul 1, 2010
Modified	Jul 1, 2010
source for spi device drivers
*/

#include "../inc/SPI.h"
#include "spi.h"

//write using selectable ports
unsigned char spiWrite( unsigned port, unsigned char i)
{
    switch(port){
        default:
            case 1: return spi1Write(i);
        #if defined(spi_v1_1) || defined (spi_v1_3)
            case 2: return spi2Write(i);
        #endif
        #if defined (spi_v1_3)
            case 3: return spi3Write(i);
        #endif
    }
}

//init Spis
void spi1Init(unsigned int prescale){
    OpenSPI1(0x0120 | prescale, 0x0000, 0x8000);
}

// send one byte of data and receive one back at the same time
unsigned char spi1Write( unsigned char i )
{
    // write to buffer for TX, wait for transfer, read
    SPI1BUF = i;
    while(!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}//spiWrite2

#if defined(spi_v1_1) || defined (spi_v1_3)
void spi2Init(unsigned int prescale){
    OpenSPI2(0x0120 | prescale, 0x0000, 0x8000);
}

// send one byte of data and receive one back at the same time
unsigned char spi2Write( unsigned char i )
{
    // write to buffer for TX, wait for transfer, read
    SPI2BUF = i;
    while(!SPI2STATbits.SPIRBF);
    return SPI2BUF;
}//spiWrite2
#endif

#if defined (spi_v1_3)
void spi3Init(unsigned int prescale){
    OpenSPI3(0x0120 | prescale, 0x0000, 0x8000);
}

// send one byte of data and receive one back at the same time
unsigned char spi3Write( unsigned char i )
{
    // write to buffer for TX, wait for transfer, read
    SPI3BUF = i;
    while(!SPI3STATbits.SPIRBF);
    return SPI3BUF;
}//spiWrite3
#endif