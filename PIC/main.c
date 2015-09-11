/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__PIC24E__)
    	#include <p24Exxxx.h>
    #elif defined (__PIC24F__)||defined (__PIC24FK__)
	#include <p24Fxxxx.h>
    #elif defined(__PIC24H__)
	#include <p24Hxxxx.h>
    #endif
#endif

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */

#include "../peripherals/inc/SPI.h"
#include "../peripherals/inc/UART1.h"

#include "../peripherals/inc/i2c-Master.h" //I hate you MPLAB
#include "app_errno.h"

#include "../peripherals/inc/MPU9150.h"
/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
//I have no clue where to put this

#define SPI_MASTER_POLPHASE0  0b00000000001 // select 16-bit master mode, CKE=1, CKP=0
#define SPI_ENABLE  0x8000 // enable SPI port, clear status
#define SPI1CLK RPOR11bits.RP22R
#define SPI1MOSI RPOR11bits.RP23R
#define UART_TX RPOR1bits.RP3R         //D10 = RP3
#define UART_RX RPOR6bits.RP12R         //D11 = RP12
int aerrno;

int16_t main(void) {
    
    aerrno = ERR_OK; /*global error var*/

   // SPI1MISO = SDI1R;
    

    //int WHOMI = 0;
    //int pulsemsg = 0;
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();

    
    __builtin_write_OSCCONL(OSCCON & ~0x40);
            //SPI
    SPI1CLK = 8; //Good luck
    SPI1MOSI = 7;
    _SDI1R = 24; //wtf input 24r
            //UART
    UART_TX = 3;
    _U1RXR = 11; //?????
    __builtin_write_OSCCONL(OSCCON | 0x40);

    spi1Init(0);
    UART1Init(9200);
    

    /*
    SPI1STAT = 0;
    SPI1CON1 = SPI_MASTER_POLPHASE0;
    SPI1CON2 = 0;//do settings
    SPI1STAT = 0; // CLEAR SPIROV
    SPI1STAT = SPI_ENABLE;
    
    TRISDbits.TRISD0 = 0; //Set D0 as output
    TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;*/
    

    //i2c_init(157); //100kHz. See data sheet
    //LATDbits.LATD0 = 1;
    //delay(30000);
    LATDbits.LATD0 = 0;

    while(1) {
        //WHOMI = i2c_read_reg(MPU9150_ADDRESS | MPU9150_WRITE,  MPU9150_WHO_AM_I);

        spi1Write(0x55);
        UART1PutChar('a');
        
        //0101 0101
        delay(1000);
        LATDbits.LATD0 = 1;
        delay(1000);
        LATDbits.LATD0 = 0;
        //WHOMI = i2c_read_reg(0b11101100, 0b10100110);
        /*if (WHOMI != 0x68) {
            ;//LATDbits.LATD0 = 1;
        }
        if (aerrno != ERR_OK) {
            for(pulsemsg = 0; pulsemsg < aerrno; pulsemsg++) {
                LATDbits.LATD0 = 1;
                delay(15);
                LATDbits.LATD0 = 0;
                delay(15);
            }
            delay(1000);
            aerrno = ERR_OK;
        }*/
        delay(1000);
        
        /*LATDbits.LATD0 = 1;
        delay(1000);
        LATDbits.LATD0 = 0;
        delay(1000);*/

    }
}
