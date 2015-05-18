/**********************************************************
 *  File: 23LC512.h
 *  Description: Header file for memory module device control
 *  Author: Albert Tate
 *  Date: 04/05/15
 **********************************************************/

 
 //Extended variable to keep track of state?

#ifndef __23LC512_H__
#define __23LC512_H__

    //Commands
    #define 23LC512_CMD_READ 0x03;
    #define 23LC512_CMD_WRITE 0x02;
    #define 23LC512_CMD_DUALIO 0x3B;
    #define 23LC512_CMD_QUADIO 0x38;
    #define 23LC512_CMD_RESETIO 0xFF;
    #define 23LC512_CMD_READMODE 0x05;
    #define 23LC512_CMD_WRITEMODE 0x01;
    
    //Modes
    #define 23LC512_MODE_BYTE 0x00;
    #define 23LC512_MODE_PAGE 0x80;
    #define 23LC512_MODE_SEQ 0x40; //default mode

    void 23LC512_init(void); //Not really needed?
    
    uint8_t 23LC512_read_mode(void);
    void 23LC512_BYTE_mode(void);
    void 23LC512_PAGE_mode(void);
    void 23LC512_SEQUENTIAL_mode(void);
    
    uint8_t 23LC512_read_byte(uint16_t addr);
    void 23LC512_read_buffer(uint16_t addr, uint16_t len, uint8_t *buff);
    
    void 23LC512_write_byte(uint16_t addr, uint8_t byte);
    void 23LC512_write_buffer(uint16_t addr, uint16_t len, uint8_t *buff);


#endif