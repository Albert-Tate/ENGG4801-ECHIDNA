/**********************************************************
 *  File: 23LC512.c
 *  Description: Implementation for memory module device control
 *  Author: Albert Tate
 *  Date: 04/05/15
 **********************************************************/

//Includes
#include "stdint.h"
#include "SPI.h"
#include "23LC512.h"

//TODO: Replace these with GPIO
//Defines
#define EXT_MEM_SET 0x01
#define EXT_MEM_RESET 0x00

//Source
uint8_t EXT_MEM_read_mode(void) {
    uint8_t rx;
    EXT_MEM_RESET;
    rx = SPI_transmit_byte(EXT_MEM_CMD_READMODE);
    EXT_MEM_SET;
	return rx;
}

void EXT_MEM_BYTE_mode(void){
    EXT_MEM_RESET;
    SPI_transmit_byte(EXT_MEM_CMD_WRITEMODE);
    SPI_transmit_byte(EXT_MEM_MODE_BYTE);
    EXT_MEM_SET;
}

void EXT_MEM_PAGE_mode(void){
    EXT_MEM_RESET;
    SPI_transmit_byte(EXT_MEM_CMD_WRITEMODE);
    SPI_transmit_byte(EXT_MEM_MODE_PAGE);
    EXT_MEM_SET;
}

void EXT_MEM_SEQUENTIAL_mode(void){
    EXT_MEM_RESET;
    SPI_transmit_byte(EXT_MEM_CMD_WRITEMODE);
    SPI_transmit_byte(EXT_MEM_MODE_SEQ);
    EXT_MEM_SET;
}


uint8_t EXT_MEM_read_byte(uint16_t addr){
    uint8_t rx;
    EXT_MEM_RESET;
    SPI_transmit_byte(EXT_MEM_CMD_READ);
    SPI_transmit_byte((uint8_t)((addr>>8)&0xFF));
    SPI_transmit_byte((uint8_t)(addr&0xFF));
    rx = SPI_transmit_byte(0x00); //TODO: Check NOP command
    EXT_MEM_SET;
    return rx;
}

void EXT_MEM_read_buffer(uint16_t addr, uint16_t len, uint8_t *buff){
    uint8_t i = 0;
    EXT_MEM_RESET;
    SPI_transmit_byte(EXT_MEM_CMD_READ);
    SPI_transmit_byte((uint8_t)((addr>>8)&0xFF));
    SPI_transmit_byte((uint8_t)(addr&0xFF));
    for (i = 0; i < len; i++){
        buff[i] = SPI_transmit_byte(0x00);
    }
    EXT_MEM_SET;
}

void EXT_MEM_write_byte(uint16_t addr, uint8_t byte){
    EXT_MEM_RESET;
    SPI_transmit_byte(EXT_MEM_CMD_WRITE);
    SPI_transmit_byte((uint8_t)((addr>>8)&0xFF)); //MSB first
    SPI_transmit_byte((uint8_t)(addr&0xFF));
    SPI_transmit_byte(byte); //TODO: Check NOP command
    EXT_MEM_SET;
}
void EXT_MEM_write_buffer(uint16_t addr, uint16_t len, uint8_t *buff){
    uint8_t i = 0;
    EXT_MEM_RESET;
    SPI_transmit_byte(EXT_MEM_CMD_WRITE);
    SPI_transmit_byte((uint8_t)((addr>>8)&0xFF));
    SPI_transmit_byte((uint8_t)(addr&0xFF));
    for (i = 0; i < len; i++){
        SPI_transmit_byte(buff[i]);
    }
}