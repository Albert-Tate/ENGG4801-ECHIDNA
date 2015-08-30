/**********************************************************
 *  File: MS5637.c
 *  Description: Driver for MS5637 Pressure/Temp Sensor
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
#include "stdint.h"
#include "MS5637.h"
#include "I2C.h"

void MS5637_READ_CALIBRATION(int16_t* buffer){ //6 16bit ints
    uint8_t i;
    I2C_send_byte(MS5637_ADDR | MS5637_READ);
    for (i = 1; i <= 6; i++) {
        I2C_send_byte(MS5637_CMD_PROM_READ | i );
        buffer[i-1] = I2C_rx_byte();
    }
}

void MS5637_START_CONVERSION(uint8_t COMMAND) {
    I2C_send_byte(MS5637_ADDR | MS5637_READ);
    I2C_send_byte(COMMAND);
}

uint32_t MS5637_READ_ADC(void){
    uint8_t i;
	uint32_t RX = 0;
    I2C_send_byte(MS5637_ADDR | MS5637_READ);
    I2C_send_byte(MS5637_CMD_ADC_READ);
    for ( i = 0; i < 3; i++) {
        RX |= I2C_rx_byte() << i;
    }
    return RX;
}

//Returns compensated data in PRESSURE and TEMP
void MS5637_CONV_METRIC(uint32_t PRESSURE_UNC, uint32_t TEMP_UNC, int16_t* CAL_DATA, int32_t* PRESSURE, int32_t* TEMP) {
    int32_t dT;
    int64_t OFF, SENS;
    
    //Calculate difference between actual and reference temp
    dT = TEMP_UNC - (CAL_DATA[4]<<8);
    *TEMP = 2000 + dT*(CAL_DATA[5]>>23);
    OFF = (CAL_DATA[1]>>17) + ((CAL_DATA[3]*dT)>>6);
    SENS = (CAL_DATA[0]>>16) + ((CAL_DATA[2]*dT)>>7);
    *PRESSURE = ((PRESSURE_UNC*(SENS>>21) - OFF)>>15);

}
    