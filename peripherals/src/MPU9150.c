/**********************************************************
 *  File: MPU9150.h
 *  Description: Header file for IMU, uses I2C
 *  Author: Albert Tate
 *  Date: 05/05/15
 **********************************************************/

#include "stdint.h"
#include "I2C.h"
//#include "../inc/i2c-Master.h" 
#include "../inc/MPU9150.h"
    
//Device is in sleep mode on boot
    
void
MPU9150_init(void)
{

    //MUST BE FIRST COMMAND ISSUED TO MPU.
    //PWR_MGMT_1 => CLKSEL = 0 (int 8MHz), SLEEP = 0, TEMP_DIS = 0 (enable temp sensor)
    MPU9150_write_byte(MPU9150_PWR_MGMT_1, 0x00);

    //disable i2c passthrough
    MPU9150_write_byte(MPU9150_INT_PIN_CFG, 0x00);

    //disable i2c master
    MPU9150_write_byte(MPU9150_USER_CTRL, 0x00);

    MPU9150_write_byte(MPU9150_CONFIG, 0x02);
        
    //SMPRT_DIV => 9 (sample rate then 1k/(1+9) = 100 Hz
    MPU9150_write_byte(MPU9150_SMPLRT_DIV, 0x09);

    //ACCEL_CONFIG => 0x00 for +-2g 
    MPU9150_write_byte(0x00, 0x00);
    
    //FIFO_EN => 0x00 (don't use FIFO)
    MPU9150_write_byte(MPU9150_FIFO_EN, 0x00);

    //PWR_MGMT_2 => 0x07 (disable gyro, keep everything else) for power
    MPU9150_write_byte(MPU9150_PWR_MGMT_2, 0x07);
}
    
uint8_t
MPU9150_read_byte(uint8_t addr)
{
    uint8_t byte;

    i2c_start();
    i2c_send_byte(MPU9150_ADDRESS | MPU9150_WRITE);
    i2c_send_byte(addr);

    i2c_start();
    i2c_send_byte(MPU9150_ADDRESS | MPU9150_READ);
    
    byte = i2c_read();
    reset_i2c_bus();

    return byte;
}

void
MPU9150_write_byte(uint8_t addr, uint8_t byte)
{
    i2c_start();
    i2c_send_byte(MPU9150_ADDRESS | MPU9150_WRITE);
    i2c_send_byte(addr);
    i2c_send_byte(byte);
    reset_i2c_bus();
    return;
}
    
void
MPU9150_read_buffer(uint8_t addr, uint8_t* buff, uint8_t len)
{
        /*NOT IMPLEMENTED*/
	return;
}
void
MPU9150_write_buffer(uint8_t addr, uint8_t* buff, uint8_t len)
{
        /*NOT IMPLEMENTED*/
	return;
}
    
//Buffer must have enough memory for 3 16 bit numbers
void
MPU9150_read_ACC(int16_t* X, int16_t* Y, int16_t* Z)
{
        *X =  MPU9150_read_byte(MPU9150_ACCEL_XOUT_L) |
                (((int16_t)MPU9150_read_byte(MPU9150_ACCEL_XOUT_H)) << 8);
        *Y =  MPU9150_read_byte(MPU9150_ACCEL_YOUT_L) | 
                (((int16_t)MPU9150_read_byte(MPU9150_ACCEL_YOUT_H)) << 8);
        *Z =  MPU9150_read_byte(MPU9150_ACCEL_ZOUT_L) |
                (((int16_t)MPU9150_read_byte(MPU9150_ACCEL_ZOUT_H)) << 8);
        return;
}
void
MPU9150_read_GYRO(int16_t* X, int16_t* Y, int16_t* Z)
{
        /*NOT IMPLEMENTED*/
	return;
}

int16_t
MPU9150_read_TEMP(void)
{ //Temp/340 + 35 to get actual value
    return MPU9150_read_byte(MPU9150_TEMP_OUT_L) |
                (((int16_t)MPU9150_read_byte(MPU9150_TEMP_OUT_H)) << 8);
}

int16_t
MPU9150_convert_TEMP(int16_t TEMP)
{
    return (int16_t) (((float)(TEMP/340)) + 35);
}

void
MPU9150_sleep(void)
{
        /*NOT IMPLEMENTED - turn off device instead*/
	return;
}

void
MPU9150_wake(void)
{
        /*NOT IMPLEMENTED*/
	return;
}
 