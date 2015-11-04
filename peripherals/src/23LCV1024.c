/**********************************************************
 *  File: 23LCV1024.c
 *  Description: Implementation for memory module device control
 *  Author: Albert Tate
 *  Date: 04/05/15
 **********************************************************/

//Includes
#include "stdint.h"
#include "SPI.h"
#include "../inc/23LCV1024.h"


//Defines
#define EXT_MEM1_SET LATGbits.LATG2 = 1;
#define EXT_MEM1_RESET LATGbits.LATG2 = 0;

#define EXT_MEM2_SET LATGbits.LATG2 = 1;
#define EXT_MEM2_RESET LATGbits.LATG2 = 0;

void 
MEM_CS(uint8_t module, uint8_t onOff)
{
    if(onOff == 1) {
        if(module > 0) {
            EXT_MEM2_SET;
        } else {
            EXT_MEM1_SET;
        }
    } else {
        if(module > 0) {
            EXT_MEM2_RESET;
        } else {
            EXT_MEM1_RESET;
        }
    }
}

uint8_t
EXT_MEM_read_byte(uint8_t module, uint32_t addr)
{
    uint8_t rx;
    MEM_CS(module, 0);
    spiWrite(0, EXT_MEM_CMD_READ);
    spiWrite(0, (uint8_t)((addr>>16)&0xFF));
    spiWrite(0, (uint8_t)((addr>>8)&0xFF));
    spiWrite(0, (uint8_t)(addr&0xFF));
    rx = spiWrite(0, 0xFF);
    MEM_CS(module, 1);
    return rx;
}

void
EXT_MEM_read_buffer(uint8_t module, uint32_t addr, uint32_t len, uint8_t *buff)
{
    uint8_t i = 0;
    MEM_CS(module, 0);;
    spiWrite(0, EXT_MEM_CMD_READ);
    spiWrite(0, (uint8_t)((addr>>16)&0xFF));
    spiWrite(0, (uint8_t)((addr>>8)&0xFF));
    spiWrite(0, (uint8_t)(addr&0xFF));
    for (i = 0; i < len; i++){
        buff[i] = spiWrite(0, 0xFF);
    }
    MEM_CS(module, 1);
}



void
EXT_MEM_write_byte(uint8_t module, uint32_t addr, uint8_t byte)
{
    MEM_CS(module, 0);
    spiWrite(0, EXT_MEM_CMD_WRITE);
    spiWrite(0, (uint8_t)((addr>>16)&0xFF));
    spiWrite(0, (uint8_t)((addr>>8)&0xFF));
    spiWrite(0, (uint8_t)(addr&0xFF));
    spiWrite(0, byte);
    MEM_CS(module, 1);
}

void
EXT_MEM_write_buffer(uint8_t module, uint32_t addr, uint32_t len, uint8_t *buff)
{
    uint8_t i = 0;
    MEM_CS(module, 0);
    spiWrite(0, EXT_MEM_CMD_WRITE);
    spiWrite(0, (uint8_t)((addr>>16)&0xFF));
    spiWrite(0, (uint8_t)((addr>>8)&0xFF));
    spiWrite(0, (uint8_t)(addr&0xFF));
    for (i = 0; i < len; i++){
        spiWrite(0, buff[i]);
    }
    MEM_CS(module, 1);
}