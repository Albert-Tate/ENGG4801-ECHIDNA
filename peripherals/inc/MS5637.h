/**********************************************************
 *  File: MS5637.h
 *  Description: Header file for Temp/Press, uses I2C
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
 
#ifndef __MS5637_H__
#define __MS5637_H__

    #define MS5637_ON command_set_gpio
    #define MS5637_OFF command_reset_gpio
    
    #define MS5637_ADDR 0b11101100
    #define MS5637_WRITE 0x00
    #define MS5637_READ 0x01
    
    #define MS5637_CMD_ADC_READ 0x00
    #define MS5637_CMD_CONV_D1_256 0x40 //Uncompensated Pressure 24 bits
    #define MS5637_CMD_CONV_D2_256 0x50 //Uncompensated Temperature 24 bits
    #define MS5637_CMD_PROM_READ 0xA0 //Or this with the next addresses

    #define MS5647_RESET 0x1E
    
    //General process: Send CMD_CONV_D1 to get the thing to get the data ready
    //Send ADC_READ to get that data ONLY AFTER IT HAS BEEN CONVERTED, EARLIER ATTEMPTS WILL CORRUPT THE DATA
    
    #define MS5637_C1 0x01
    #define MS5637_C2 0x02
    #define MS5637_C3 0x03
    #define MS5637_C4 0x04
    #define MS5637_C5 0x05
    #define MS5637_C6 0x06
    

    void MS5637_READ_CALIBRATION(uint16_t* buffer); //6 16bit ints
    void MS5637_START_CONVERSION(uint8_t COMMAND);
    uint32_t MS5637_READ_ADC(void);
    
    //Returns compensated data in PRESSURE_UNC and TEMP_UNC
    void MS5637_CONV_METRIC(uint32_t PRESSURE_UNC, uint32_t TEMP_UNC, uint16_t* CAL_DATA, int32_t* PRESSURE, int32_t* TEMP);
    
    

#endif