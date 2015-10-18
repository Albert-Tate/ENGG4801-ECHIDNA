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
#include "../peripherals/inc/23LCV1024.h"

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

#define MAX17040_ADDRESS 0b01101100

#define LED_ON LATDbits.LATD9 = 1;
#define LED_OFF LATDbits.LATD9 = 0;

struct TIME {
    uint8_t sec;
    uint8_t min;
    uint8_t hr;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} local_time;

#define MAX_MEASURE 3
struct MEASUREMENT {
    //MS5637
    int32_t TEMPERATURE;
    int32_t PRESSURE;
    //Accelerometery
    int16_t X_ACC;
    int16_t Y_ACC;
    int16_t Z_ACC;
    //Light sensor
    uint16_t LALOG;
    //Time
    uint8_t day; //0-255 (I wish)
    uint8_t hour; //lots of bad values could happen here
    uint8_t minute;
    uint8_t second;
    //TOTAL 160 bit (1Mbit = 2^20, 2^20/144 = 7281 across 2 thingys = 14652 = 4 hours of sampling at 1hz. pretty neato
    //Debug: Battery remaining (0-10 000)
    //int16_t BATT;
    //now 172 bit
} measure[MAX_MEASURE];



void UART_PUTVAR(const char* name, int namelen, int value);
void RTC_INIT(void);
void RTC_SNAPSHOT(struct TIME*);
void RTC_ALARMSET(uint8_t);
void RTC_ALARMOFF(void);
uint16_t MAX17040_SOC(void);
uint16_t MAX17040_VDD(void);

int aerrno;
int meas_index = 0;
int mem_pointer = 0;
int BATT_SOC = 1000;
uint16_t MSCAL[6];
uint32_t PRES = 0;
uint32_t TEMP = 0;

int16_t main(void) {

    aerrno = ERR_OK; /*global error var*/
/*    int before1, before2, before3,after1,after2,after3;
    before1 = 0xAC;
    before2 = 0xF1;
    before3 = 0xB3;*/
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    //setup remappable peripherals
    __builtin_write_OSCCONL(OSCCON & ~0x40);
            //SPI
    SPI1CLK = 8; //Good luck Matches altium 
    SPI1MOSI = 7;
    _SDI1R = 24; //wtf input 24r
            //UART
    UART_TX = 3; //D10
    _U1RXR = 11; //D11
    __builtin_write_OSCCONL(OSCCON | 0x40); 


    spi1Init(0);
    //UART1Init(6); //Gives 142000 Baud rate. Clock at approx 20Mhz (10mHz FCY)
    
    RTC_INIT();
    i2c_init(157); //100kHz. See data sheet 157
    ADC_init();

    MS5637_INIT_PIN; //Set F0 as output (MS5VCC)
    MPU9150_INIT_PIN; //Set D6 as output (MPU9150)
    TRISDbits.TRISD9 = 0; // ULED
    TRISDbits.TRISD7 = 0; //set photosensor power pin as output

    TRISGbits.TRISG2 = 0; //Mem1
    TRISGbits.TRISG3 = 0; //Mem2
    LATGbits.LATG2 = 1; //CS high
    LATGbits.LATG3 = 1; //CS high

    MS5637_ON;
    delay_us_3(1000);
    MS5637_READ_CALIBRATION(MSCAL);

    RTC_ALARMSET(1); //every second, see pg 280 "ALCFGRPT" register. NOT 1:1
    
    while(1) {
        MS5637_OFF;
        MPU9150_OFF;
        LATDbits.LATD7 = 0;


/*        LATGbits.LATG2 = 0; //Pull CS low
        //write before to pos 0
        spiWrite(0, 0x02); //Write command
        spiWrite(0, 0x00); //Addr MSB
        spiWrite(0, 0x00); //Addr middle
        spiWrite(0, 0x00); //Addr LSB
        spiWrite(0, before1);
        spiWrite(0, before2);
        spiWrite(0, before3);
        LATGbits.LATG2 = 1;

        delay_us_3(10000);

        //Read pos 0 to after
        LATGbits.LATG2 = 0; //Pull CS low
        //read before from pos 0
        spiWrite(0, 0x03); //read command
        spiWrite(0, 0x00); //Addr MSB
        spiWrite(0, 0x00); //Addr middle
        spiWrite(0, 0x00); //Addr LSB
        after1 = spiWrite(0, 0xFF);
        after2 = spiWrite(0, 0xFF);
        after3 = spiWrite(0, 0xFF);
        LATGbits.LATG2 = 1;*/

        //Pressure sensor
        MS5637_ON;
        MPU9150_ON;
        delay_us_3(100);
        if (MSCAL[0] == 0 || MSCAL[0] == 0xFFFF) {
            MS5637_READ_CALIBRATION(MSCAL);
            if (aerrno) aerrno = ERR_OK;
        }
        
        MS5637_START_CONVERSION(MS5637_CMD_CONV_D1_256);
        delay_us_3(10000); //fine tune these
        delay_us_3(10000);
        PRES = MS5637_READ_ADC();

        MS5637_START_CONVERSION(MS5637_CMD_CONV_D2_256);
        delay_us_3(10000);
        delay_us_3(10000);
        TEMP = MS5637_READ_ADC();

        MS5637_CONV_METRIC(PRES , TEMP, MSCAL, 
                &measure[meas_index].PRESSURE,
                &measure[meas_index].TEMPERATURE);
        
        MPU9150_init();
        //Now give MPU time to stabilise

        delay_us_3(65535);
        delay_us_3(65535);

        MPU9150_read_ACC(&(measure[meas_index].X_ACC),
                         &(measure[meas_index].Y_ACC),
                         &(measure[meas_index].Z_ACC));
        //dont bother with mputemp, less accurate and I don't want to kalman

        MPU9150_OFF;
        MS5637_OFF; //This needs to be on for MPU9150 to work, silicon bug

        LATDbits.LATD7 = 1;
        delay_us_3(5000); //4ms
        measure[meas_index].LALOG = ADCSample();
        LATDbits.LATD7 = 0;

        BATT_SOC = MAX17040_SOC();
        if(aerrno) {
            BATT_SOC = -1; //Unknown battery state
        }

        //load date, time etc
        RTC_SNAPSHOT(&local_time);
        measure[meas_index].day = local_time.day;
        measure[meas_index].hour = local_time.hr;
        measure[meas_index].minute = local_time.min;
        measure[meas_index].second = local_time.sec;

        meas_index++;
        /*
         If we have all measurements we can fit locally:
         Attempt to write to SRAM
         If no room on either SRAM, write to SD Card
         */
        if(meas_index == MAX_MEASURE) {
            uint32_t i;
            RTC_ALARMOFF(); //No interruptions while this happens

            //Think harder about what happens near the limits of size
            if(meas_index + mem_pointer > EXT_MEM_SIZE) {
                //write to device 1
                for(i = 0; i < meas_index; i++) {
                    EXT_MEM_write_buffer(0, mem_pointer, sizeof(struct MEASUREMENT),
                            (uint8_t*)&(measure[i]));
                    mem_pointer += sizeof(struct MEASUREMENT);
                }
            } else if(meas_index + mem_pointer < 2*EXT_MEM_SIZE) {
                for(i = 0; i < meas_index; i++) {
                    EXT_MEM_write_buffer(1, mem_pointer - EXT_MEM_SIZE, sizeof(struct MEASUREMENT),
                            (uint8_t*)&(measure[i]));
                    mem_pointer += sizeof(struct MEASUREMENT);
                }
            } else {
                uint64_t j;
                //SD CARD
                /*EXT_MEM_SIZE/sizeof(struct MEASUREMENT) = Number of measure
                  structs that fit inside this bad boy
                 measstructno/MAX_MEASURE = how many times we have to fill
                 measure struct array*/

                //Read structs in from mem1, cpy to SD CARD
                for(j = 0;
                     j < (EXT_MEM_SIZE/sizeof(struct MEASUREMENT)/MAX_MEASURE);
                     j++) {
                    for(i = 0; i < MAX_MEASURE; i++) {
                        EXT_MEM_read_buffer(0, i + j*MAX_MEASURE,
                                sizeof(struct MEASUREMENT),
                                (uint8_t*)&(measure[i]));
                    }
                    for(i = 0; i < MAX_MEASURE; i++) {
                        Nop(); //WRITE TO SD CARD!!!!!!
                    }
                }
                //Read structs in from mem2, cpy to SD CARD
                for(j = 0;
                     j < (EXT_MEM_SIZE/sizeof(struct MEASUREMENT)/MAX_MEASURE);
                     j++) {
                    for(i = 0; i < MAX_MEASURE; i++) {
                        EXT_MEM_read_buffer(1, i + j*MAX_MEASURE,
                                sizeof(struct MEASUREMENT),
                                (uint8_t*)&(measure[i]));
                    }
                    for(i = 0; i < MAX_MEASURE; i++) {
                        Nop(); //WRITE TO SD CARD!!!!!!
                    }
                }
            }           
            meas_index = 0;
            Nop();
            Nop();
            RTC_ALARMSET(1);
        }

        ENTER_SLEEP;
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
RTC_ALARMOFF(void)
{
    _ALRMEN = 0;
    ALRMVAL = 0;
    _ARPT = 0;
    _CHIME = 0;

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


uint16_t
MAX17040_SOC(void)
{
    uint8_t low, high;
    i2c_start();
    i2c_send_byte(MAX17040_ADDRESS | 0x00);
    i2c_send_byte(0x04); //SOC register 0x04

    i2c_start();
    i2c_send_byte(MAX17040_ADDRESS | 0x01);
    high = i2c_read_ack();
    low = i2c_read();
    reset_i2c_bus();

    return ((uint16_t)high << 8) | low;
}

uint16_t
MAX17040_VDD(void)
{
    uint8_t low, high;
    i2c_start();
    i2c_send_byte(MAX17040_ADDRESS | 0x00);
    i2c_send_byte(0x02); //SOC register

    i2c_start();
    i2c_send_byte(MAX17040_ADDRESS | 0x01);
    high = i2c_read_ack();
    low = i2c_read();
    reset_i2c_bus();

    return ((uint16_t)high << 8) | low;
}