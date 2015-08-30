#include "stdint.h"
#include "i2c.h"
#include <stdio.h>

void i2c_init(int BRG)
{
    int temp;

    // initialize ports
    TRISGbits.TRISG2 = 1;
    TRISGbits.TRISG3 = 1;

    I2C1BRG = BRG;
    I2C1CONbits.I2CEN = 0;
    I2C1CONbits.DISSLW = 1;
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.I2CEN = 1;
    temp = I2CRCV;
    reset_i2c_bus();
}

void I2C_send_byte(uint8_t byte) {
	return;
}
uint8_t I2C_rx_byte(void){
	return 0;
}