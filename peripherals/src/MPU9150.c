/**********************************************************
 *  File: MPU9150.h
 *  Description: Header file for IMU, uses I2C
 *  Author: Albert Tate
 *  Date: 05/05/15
 **********************************************************/

#include "stdint.h"
#include "I2C.h"
#include "../inc/i2c-Master.h"
#include "../inc/MPU9150.h" //I blame MPLABX Entirely
    
//Start -> TX 8 bits(ADDR) -> ACK -> More Bytes -> Ack etc -> STOP
//Start -> TX 8 bits -> ACK -> STOP
//Device is in sleep mode on boot
    
void
MPU9150_init(void)
{
    //DLPF => 1k (CONFIG => 0x02)
    //i2c_send_byte(MPU9150_ADDRESS);
    //i2c_send_byte(MPU9150_WRITE | MPU9150_CONFIG);
    //i2c_send_byte(0x02);
    MPU9150_write_byte(MPU9150_CONFIG, 0x02);
        
    //SMPRT_DIV => 9 (sample rate then 1k/(1+9) = 100 Hz
    MPU9150_write_byte(MPU9150_SMPLRT_DIV, 0x09);

    //ACCEL_CONFIG => 0x00 for +-2g (ADD FUNCTIONS TO CHANGE THIS!!!!!)
    MPU9150_write_byte(0x00, 0x00);
    
    //FIFO_EN => 0x00 (don't use FIFO)
    MPU9150_write_byte(MPU9150_FIFO_EN, 0x00);

    //PWR_MGMT_1 => CLKSEL = 0 (int 8MHz), SLEEP = 0, TEMP_DIS = 0 (enable temp sensor)
    MPU9150_write_byte(MPU9150_PWR_MGMT_1, 0x00);

    //PWR_MGMT_2 => 0x07 (disable gyro, keep everything else)
    MPU9150_write_byte(MPU9150_PWR_MGMT_2, 0x07);
}
    
uint8_t
MPU9150_read_byte(uint8_t addr)
{
	return 0;
}

void
MPU9150_write_byte(uint8_t addr, uint8_t byte)
{
	return;
}
    
void
MPU9150_read_buffer(uint8_t addr, uint8_t* buff, uint8_t len)
{
	return;
}
void
MPU9150_write_buffer(uint8_t addr, uint8_t* buff, uint8_t len)
{
	return;
}
    
//Buffer must have enough memory for 3 16 bit numbers
void
MPU9150_read_ACC(int16_t* buff)
{
        MPU9150_read_buffer(MPU9150_ACCEL_XOUT_H, buff, 6);
        return;
}
void
MPU9150_read_GYRO(int16_t* buff)
{
	return;
}

int16_t
MPU9150_read_TEMP(void)
{ //Temp/340 + 35 to get actual value
    return 0;
}

void
MPU9150_sleep(void)
{
	return;
}

void
MPU9150_wake(void)
{
	return;
}
 