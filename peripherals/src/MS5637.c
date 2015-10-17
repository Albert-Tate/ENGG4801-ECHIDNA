/**********************************************************
 *  File: MS5637.c
 *  Description: Driver for MS5637 Pressure/Temp Sensor
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
#include "stdint.h"
#include "../../PIC/system.h"
#include "../inc/MS5637.h"
//#include "../inc/i2c-Master.h" //Only works when implicitly called.
//Literally no one on earth can describe this
#include "I2C.h"

void MS5637_READ_CALIBRATION(uint16_t* buffer){ //6 16bit ints
    uint8_t i;
    uint16_t temp;

    for (i = 0; i <= 5; i++) {
        i2c_start();
        i2c_send_byte(MS5637_ADDR | MS5637_WRITE);
        i2c_send_byte(MS5637_CMD_PROM_READ + ((i+1)*2));
        reset_i2c_bus();

        delay_us_3(3);
        i2c_start();
        i2c_send_byte(MS5637_ADDR | MS5637_READ);
        temp = i2c_read();
        buffer[i] = (temp << 8) | i2c_read();
        reset_i2c_bus();
    }
    //reset_i2c_bus();
}

void MS5637_START_CONVERSION(uint8_t COMMAND) {
    i2c_start();
    i2c_send_byte(MS5637_ADDR | MS5637_WRITE);
    i2c_send_byte(COMMAND);
    reset_i2c_bus();
}

uint32_t MS5637_READ_ADC(void){
    uint8_t i;
    uint32_t temp;
    uint32_t RX = 0;
    i2c_start();
    i2c_send_byte(MS5637_ADDR | MS5637_WRITE);
    i2c_send_byte(MS5637_CMD_ADC_READ);
    reset_i2c_bus();
    delay_us_3(3);

    i2c_start();
    i2c_send_byte(MS5637_ADDR | MS5637_READ);
    for(i = 0; i < 2; i++) {
        temp = i2c_read_ack();
        RX |= temp << ((2-i)*8);
    }
    temp = i2c_read();
    RX |= temp;
    
    reset_i2c_bus();
    return RX;
}

//Returns compensated data in PRESSURE and TEMP
void MS5637_CONV_METRIC(uint32_t PRESSURE_UNC, uint32_t TEMP_UNC, uint16_t* CAL_DATA, int32_t* PRESSURE, int32_t* TEMP) {
    int32_t dT;
    int64_t OFF, SENS;
    //This is pretty intense for no apparent reason Also the datasheet is wrong
    //Calculate difference between actual and reference temp
    dT = TEMP_UNC - ( ((int32_t)CAL_DATA[4]) << 8);
    *TEMP = 2000 + (((int64_t)dT* ((int64_t)CAL_DATA[5])) >> 23);
    
    OFF = ( ((int64_t)CAL_DATA[1])<<17) + ((CAL_DATA[3]*dT)>>6);
    SENS = ( ((int64_t)CAL_DATA[0])<<16) + ((CAL_DATA[2]*dT)>>7);
    *PRESSURE = ((PRESSURE_UNC*(SENS>>21) - OFF)>>15);
}
