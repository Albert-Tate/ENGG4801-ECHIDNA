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
#include "stdio.h"

#include "../peripherals/inc/SPI.h"
#include "../peripherals/inc/UART1.h"
#include "P24ADC.h"

#include "../peripherals/inc/i2c-Master.h" //I hate you MPLAB
#include "app_errno.h"

#include "../peripherals/inc/MPU9150.h"
#include "../peripherals/inc/MS5637.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
//I have no clue where to put this
// .equiv SLEEP_MODE, 0x0000
//.equiv IDLE_MODE, 0x0001

#define SPI_MASTER_POLPHASE0  0b00000000001 // select 16-bit master mode, CKE=1, CKP=0
#define SPI_ENABLE  0x8000 // enable SPI port, clear status
#define SPI1CLK RPOR11bits.RP22R
#define SPI1MOSI RPOR11bits.RP23R
#define UART_TX RPOR1bits.RP3R         //D10 = RP3
#define UART_RX RPOR6bits.RP12R         //D11 = RP12

#define ENTER_SLEEP asm volatile("PWRSAV #0x0000");

#define ENTER_LV_SLEEP _RETEN = 1; \
                       asm volatile("PWRSAV #0x0000");

#define ENTER_DEEP_SLEEP asm volatile("CLR DSCON"); \
                         asm volatile("CLR DSCON"); \
                         asm volatile("BSET DSCON, #DSEN"); \
                         asm volatile("BSET DSCON, #DSEN"); \
                         asm volatile("PWRSAV, #0x0000");

struct TIME {
    uint8_t sec;
    uint8_t min;
    uint8_t hr;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} local_time;

int aerrno;
uint16_t MSCAL[6];
void UART_PUTVAR(const char* name, int namelen, int value);
void RTC_INIT(void);
void RTC_SNAPSHOT(struct TIME*);
void RTC_ALARMSET(uint8_t);

int16_t main(void) {
    //uint8_t WHOMI = 0;
    uint32_t PRES = 0;
    uint32_t TEMP = 0;
    int32_t MET_PRES = 0;
    int32_t MET_TEMP = 0;
    //uint16_t RTC_REG;
    //int8_t sec;
    //int8_t min;
    //long x = 0;
    //CHECK IF DPSLP IS SET AND DO DIFFERENT THINGS!

    aerrno = ERR_OK; /*global error var*/
    
    //int pulsemsg = 0;
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /*__builtin_write_OSCCONL(OSCCON & ~0x40);
            //SPI
    SPI1CLK = 8; //Good luck
    SPI1MOSI = 7;
    _SDI1R = 24; //wtf input 24r
            //UART
    UART_TX = 3; //D10
    _U1RXR = 11; //D11
    __builtin_write_OSCCONL(OSCCON | 0x40); */

    //spi1Init(0);
    //UART1Init(6); //Gives 142000 Baud rate. Clock at approx 20Mhz (10mHz FCY)
    //UART1PutChar('S');
    
    //RTC_INIT();
    TRISFbits.TRISF0 = 0; //Set F0 as output (MS5VCC)
    TRISDbits.TRISD9 = 0; // ULED
    TRISDbits.TRISD6 = 0; //Set D6 as output (MPUVCC)
    //TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;
    //RTC_ALARMSET(1);
    //ADC_init();
    i2c_init(157); //100kHz. See data sheet
    //i2c_init(357);
    //LATDbits.LATD0 = 1;
    //delay(30000);
    //LATDbits.LATD0 = 0;

    //LATDbits.LATD9 = 1; //LED
    
    LATDbits.LATD6 = 1; //Turn on accelerometer
    LATFbits.LATF0 = 1; //Turn on MS5637 Pressure sensor
    //i2c_write(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x01);
    MS5637_START_CONVERSION(MS5647_RESET);
    while(1) {
        //WHOMI = i2c_read_reg(MPU9150_ADDRESS, MPU9150_WHO_AM_I);
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
        Nop();
        PRES = 0;

        //x = ADCSample();
        //UART_PUTVAR("ADC", 3, ((x*3300) / 1024));
        //i2c_command(MS5637_ADDR,  MS5647_RESET);
        /*RTC_SNAPSHOT(&local_time);
        UART_PUTVAR("yr", 2, local_time.year);
        UART_PUTVAR("mon", 3, local_time.month);
        UART_PUTVAR("day", 3, local_time.day);
        UART_PUTVAR("hr", 2, local_time.hr);
        UART_PUTVAR("min", 3, local_time.min);
        UART_PUTVAR("sec", 3, local_time.sec);
        UART1PutChar('\n');*/
        //ENTER_SLEEP;
        //delay(1000);
        //LATDbits.LATD0 = 1;
        //delay(1000);
        //LATDbits.LATD0 = 0;
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
        //delay(1000);
        
        /*LATDbits.LATD0 = 1;
        delay(1000);
        LATDbits.LATD0 = 0;
        delay(1000);*/

    }
}

void
UART_PUTVAR(const char* name, int namelen, int value)
{
    int i, j;
    char buf[10];
    for(i = 0; i < namelen; i++) {
        UART1PutChar(name[i]);
    }
    UART1PutChar(' ');
    j = snprintf(buf, 10, "%d", value);
    for(i = 0; i < j; i++) {
        UART1PutChar(buf[i]);
    }
    UART1PutChar('\n');
    UART1PutChar('\r'); //Goddamit windows
}

void
RTC_SNAPSHOT(struct TIME* time)
{
    int16_t RTC_REG;
    _RTCPTR = 3;

    RTC_REG = RTCVAL;
    time->year = (((RTC_REG&0xFF) >> 4)*10) + (((RTC_REG)&0x0F) % 10);

    RTC_REG = RTCVAL;
    time->month = (((RTC_REG >> 8) >> 4)*10) + (((RTC_REG >> 8)&0x0F) % 10);
    time->day = (((RTC_REG&0xFF) >> 4)*10) + (((RTC_REG)&0x0F) % 10);

    RTC_REG = RTCVAL;
    // weekday TIME->day = (((RTC_REG >> 8) >> 4)*10) + (((RTC_REG >> 8)&0x0F) % 10);
    time->hr = (((RTC_REG&0xFF) >> 4)*10) + (((RTC_REG)&0x0F) % 10);

    RTC_REG = RTCVAL;
    time->min = (((RTC_REG >> 8) >> 4)*10) + (((RTC_REG >> 8)&0x0F) % 10);
    time->sec = (((RTC_REG&0xFF) >> 4)*10) + (((RTC_REG)&0x0F) % 10);
}

void
RTC_ALARMSET(uint8_t interval)
{
    _ALRMEN = 0;

    ALRMVAL = 0; //Every  half second (AMASK3:0)
    _AMASK = interval;

    _ARPT = 0; //Once
    _CHIME = 1; //Never stops

    _ALRMEN = 1;
    _RTCIF = 0;
    _RTCIE = 1; //Enable interrupts
    
}

void
RTC_INIT(void)
{
    asm volatile("push w7"); //I think having to write ASM is fine
    asm volatile("push w8");
    asm volatile("disi #5");
    asm volatile("mov #0x55, w7");
    asm volatile("mov w7, _NVMKEY");
    asm volatile("mov #0xAA, w8");
    asm volatile("mov w8, _NVMKEY");
    asm volatile("bset _RCFGCAL, #13"); //set the RTCWREN bit
    asm volatile("pop w8");
    asm volatile("pop w7");

    _RTCPTR = 3;
    RTCVAL = 0x2015;
    RTCVAL = 0x1106; //6 nov
    RTCVAL = 0x0510;
    RTCVAL = 0x0000;

    RTCPWC = (0b01 << 10) | 0b01 << 0 ;
    RCFGCAL = 0b1 << 15 | 0b1 << 13;

    _RTCWREN = 0;
}