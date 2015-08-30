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
    #define EXT_MEM_CMD_READ 0x03
    #define EXT_MEM_CMD_WRITE 0x02
    #define EXT_MEM_CMD_DUALIO 0x3B
    #define EXT_MEM_CMD_QUADIO 0x38
    #define EXT_MEM_CMD_RESETIO 0xFF
    #define EXT_MEM_CMD_READMODE 0x05
    #define EXT_MEM_CMD_WRITEMODE 0x01
    
    //Modes
    #define EXT_MEM_MODE_BYTE 0x00
    #define EXT_MEM_MODE_PAGE 0x80
    #define EXT_MEM_MODE_SEQ 0x40 //default mode

    void EXT_MEM_init(void); //Not really needed?
    
    uint8_t EXT_MEM_read_mode(void);
    void EXT_MEM_BYTE_mode(void);
    void EXT_MEM_PAGE_mode(void);
    void EXT_MEM_SEQUENTIAL_mode(void);
    
    uint8_t EXT_MEM_read_byte(uint16_t addr);
    void EXT_MEM_read_buffer(uint16_t addr, uint16_t len, uint8_t *buff);
    
    void EXT_MEM_write_byte(uint16_t addr, uint8_t byte);
    void EXT_MEM_write_buffer(uint16_t addr, uint16_t len, uint8_t *buff);


#endif