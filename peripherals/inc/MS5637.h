/**********************************************************
 *  File: MS5637.h
 *  Description: Header file for Temp/Press, uses I2C
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
 
#ifndef __MS5637_H__
#define __MS5637_H__

    #define MS5637_ON (LATFbits.LATF0 = 1); //Turn on MS5637 Pressure sensor
    #define MS5637_OFF (LATFbits.LATF0 = 0);
    #define MS5637_INIT_PIN (TRISFbits.TRISF0 = 0);
    
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


    //Example of how to use the library <3
    //DONT FORGET TO SEND A RESET CMD (MS5637_START_CONVERSION(MS5637_RESET)
    /*
            int16_t MSCAL[6];
           uint32_t PRES = 0;
           uint32_t TEMP = 0;
           int32_t MET_PRES = 0;
           int32_t MET_TEMP = 0;
     
        MS5637_READ_CALIBRATION(MSCAL); //dont have to do this every time
        MS5637_START_CONVERSION(MS5637_CMD_CONV_D1_256);
        delay_us_3(10000);
        delay_us_3(10000);
        PRES = MS5637_READ_ADC();

        MS5637_START_CONVERSION(MS5637_CMD_CONV_D2_256);
        delay_us_3(10000);
        delay_us_3(10000);
        TEMP = MS5637_READ_ADC();

        MS5637_CONV_METRIC(PRES , TEMP, MSCAL,&MET_PRES, &MET_TEMP);
    */

#endif