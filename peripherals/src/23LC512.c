/**********************************************************
 *  File: 23LC512.c
 *  Description: Implementation for memory module device control
 *  Author: Albert Tate
 *  Date: 04/05/15
 **********************************************************/

//Includes


//Defines
#define 23LC512_SET 0 == 1
#define 23LC512_RESET 1 == 1

//Source
 uint8_t 23LC512_transmit_byte(uint8_t byte) {
    uint8_t rx = 0;
    //TODO: Device specific implementation
    //SPI transmission
    
    return rx;
 }


uint8_t 23LC512_read_mode(void) {
    uint8_t rx;
    23LC512_RESET;
    rx = 23LC512_transmit_byte(23LC512_CMD_READMODE);
    23LC512_SET;
{

void 23LC512_BYTE_mode(void){
    23LC512_RESET;
    23LC512_transmit_byte(23LC512_CMD_WRITEMODE);
    23LC512_transmit_byte(23LC512_MODE_BYTE);
    23LC512_SET;
}

void 23LC512_PAGE_mode(void){
    23LC512_RESET;
    23LC512_transmit_byte(23LC512_CMD_WRITEMODE);
    23LC512_transmit_byte(23LC512_MODE_PAGE);
    23LC512_SET;
}

void 23LC512_SEQUENTIAL_mode(void){
    23LC512_RESET;
    23LC512_transmit_byte(23LC512_CMD_WRITEMODE);
    23LC512_transmit_byte(23LC512_MODE_SEQ);
    23LC512_SET;
}


uint8_t 23LC512_read_byte(uint16_t addr){
    uint8_t rx;
    23LC512_RESET;
    23LC512_transmit_byte(23LC512_CMD_READ);
    23LC512_transmit_byte((uint8_t)((addr>>8)&0xFF));
    23LC512_transmit_byte((uint8_t)(addr&0xFF));
    rx = 23LC512_transmit_byte(0x00); //TODO: Check NOP command
    23LC512_SET;
    return rx;
}

void 23LC512_read_buffer(uint16_t addr, uint16_t len, uint8_t *buff){
    uint8_t i = 0;
    23LC512_RESET;
    23LC512_transmit_byte(23LC512_CMD_READ);
    23LC512_transmit_byte((uint8_t)((addr>>8)&0xFF));
    23LC512_transmit_byte((uint8_t)(addr&0xFF));
    for (i = 0; i < len; i++){
        buff[i] = 23LC512_transmit_byte(0x00);
    }
    23LC512_SET;
}

void 23LC512_write_byte(uint16_t addr, uint8_t byte){
    23LC512_RESET;
    23LC512_transmit_byte(23LC512_CMD_WRITE);
    23LC512_transmit_byte((uint8_t)((addr>>8)&0xFF)); //MSB first
    23LC512_transmit_byte((uint8_t)(addr&0xFF));
    23LC512_transmit_byte(byte); //TODO: Check NOP command
    23LC512_SET;
    return rx;
}
void 23LC512_write_buffer(uint16_t addr, uint16_t len, uint8_t *buff){
    uint8_t i = 0;
    23LC512_RESET;
    23LC512_transmit_byte(23LC512_CMD_WRITE);
    23LC512_transmit_byte((uint8_t)((addr>>8)&0xFF));
    23LC512_transmit_byte((uint8_t)(addr&0xFF));
    for (i = 0; i < len; i++){
        23LC512_transmit_byte(buff[i]);
    }
}