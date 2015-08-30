/**********************************************************
 *  File: I2C.h
 *  Description: Driver for I2C
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
 
#ifndef __I2C_H__
#define __I2C_H__

void I2C_send_byte(uint8_t byte);
uint8_t I2C_rx_byte(void);


#endif