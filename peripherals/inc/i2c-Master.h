/**********************************************************
 *  File: I2C.h
 *  Description: Driver for I2C
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
 
#ifndef __I2C_H__
#define __I2C_H__

void i2c_init(int BRG);
void i2c_start(void);
void i2c_restart(void);
void reset_i2c_bus(void);

char i2c_read(void);
char i2c_read_ack(void);

char i2c_send_byte(int data);

void i2c_write(char addr, char subaddr, char value);
char i2c_read_reg(char addr, char subaddr);
void i2c_command(char addr, char command);
char i2c_poll(char addr);

#endif