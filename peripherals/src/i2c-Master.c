#include "stdint.h"
#include "i2c.h"
#include <p24Fxxxx.h>
#include <stdio.h>

#include "../../PIC/system.h"
#include "../inc/i2c-Master.h"
#include "../../PIC/app_errno.h"

void 
i2c_init(int BRG)
{
    int temp;

    // initialize ports (i2c2)
    TRISFbits.TRISF4 = 1;
    TRISFbits.TRISF5 = 1;

    I2C2BRG = BRG;
    I2C2CONbits.I2CEN = 0;
    I2C2CONbits.DISSLW = 1;
    IFS3bits.MI2C2IF = 0;
    I2C2CONbits.I2CEN = 1;
    temp = I2C2RCV;
    reset_i2c_bus();
}

void
i2c_start(void)
{
    int x = 0;
    /*
     * Some versions of the PIC24F chip have a silicon bug preventing a
     * proper start condition being issues on I2C bus. Here is a bit bashed
     * workaround if you get one of those chips
     */
    /*I2C1CONbits.I2CEN = 0;	//disable IIC
    delay_us_3(10); 
    TRISFbits.TRISF4 = 0; 
    LATFbits.LATF4 = 0; 
    delay_us_3(10); 
    LATFbits.LATF4 = 1; 
    delay_us_3(10);*/

    I2C2CONbits.ACKDT = 0; //Reset ack
    delay_us_3(10);
    I2C2CONbits.SEN = 1; //Start con
    Nop(); //Do not remove, required for start condition

    while(I2C2CONbits.SEN)
    {
        delay_us_3(1);
        x++;
        if(x > 20) {
            aerrno = I2CSTART;
            break;
        }
    }
    delay_us_3(2);
}

void
i2c_restart(void)
{
    int x = 0;
    I2C2CONbits.RSEN = 1; //Restart cond.
    Nop();

    while(I2C2CONbits.RSEN)
    {
        delay_us_3(1);
        x++;
        if(x > 20) {
            aerrno = I2CSTART;
            break;
        }
    }
    delay_us_3(2);
}

void
reset_i2c_bus(void)
{
    int x = 0;
    I2C2CONbits.PEN = 1;
    while(I2C2CONbits.PEN)
    {
        delay_us_3(1);
        x++;
        if(x > 20) {
            aerrno = I2CSTOP;
            break;
        }
    }
    //reset status
    I2C2CONbits.RCEN = 0;
    IFS3bits.MI2C2IF = 0;
    I2C2STATbits.IWCOL = 0;
    I2C2STATbits.BCL = 0;
    delay_us_3(10);
}

char
i2c_send_byte(int data)
{
    int i;
    while(I2C2STATbits.TBF) {}
    IFS3bits.MI2C2IF = 0;
    I2C2TRN = data;

    for (i = 0; i<500; i++)
    {
        if (!I2C2STATbits.TRSTAT) break;
        delay_us_3(3);
    }
    if(i == 500) {
        aerrno = I2CSENDTIMEOUT;
        return (1);
    }
    
    if(I2C2STATbits.ACKSTAT == 1) {
        aerrno = I2CACK; //Ack failed to receive
        return(1);
    }
    
    delay_us_3(1);
    return (0);
}

char
i2c_read(void)
{
    int i = 0;
    char data = 0;

    I2C2CONbits.RCEN = 1;

    while(!I2C2STATbits.RBF) {
        i++;
        if (i > 2000) {
            aerrno = I2CRECTIMEOUT;
            break;
        }
    }

    data = I2C2RCV;

    return data;
}

char
i2c_read_ack(void)
{
    char data = 0;
    data = i2c_read();

    I2C2CONbits.ACKEN = 1;
    delay_us_3(10);

    return data;
}

void
i2c_write(char addr, char subaddr, char value)
{
   i2c_start();
   i2c_send_byte(addr);
   i2c_send_byte(subaddr);
   i2c_send_byte(value);
   reset_i2c_bus();
}

void
i2c_command(char addr, char command) {
    i2c_start();
    i2c_send_byte(addr);
    i2c_send_byte(command);
    reset_i2c_bus();
}

char
i2c_read_reg(char addr, char subaddr)
{
   char temp;

   i2c_start();
   i2c_send_byte(addr);
   i2c_send_byte(subaddr);
   delay_us_3(3);
   
   i2c_restart();
   i2c_send_byte(addr | 0x01);
   temp = i2c_read();

   reset_i2c_bus();
   return temp;
}

//returns 1 on failure
char
i2c_poll(char addr)
{
   unsigned char temp = 0;

   i2c_start();
   temp = i2c_send_byte(addr);
   reset_i2c_bus();

   return temp;
}