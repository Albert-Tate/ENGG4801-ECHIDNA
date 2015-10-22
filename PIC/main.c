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


#include "../peripherals/ff11a/src/diskio.h"
#include "../peripherals/ff11a/src/ff.h"

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
#define UART_RX RPINR18bits.U1RXR         //D11 = RP12

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

#define SD_OFF    LATDbits.LATD8 = 1; //drive high (out low)
#define SD_ON     LATDbits.LATD8 = 0;

#define TIMER_START     T1CON = 0b1000000000000000;
#define TIMER_END       T1CON = 0b0000000000000000;

//#define SD_CARD 0x00 //enables SD card


struct TIME {
    uint8_t sec;
    uint8_t min;
    uint8_t hr;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} local_time;

#define MAX_MEASURE 60 //How many fit on device before we write to SRAM
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
void TimerInit(void);
void init_ports(void);
int UART_SEND_PACKET(void);

//_timer interrupt for FatFS 1k/hz Found in mmc_pic24f.c
void __attribute__((__interrupt__)) _T1Interrupt(void);

int aerrno;
int meas_index = 0;
int mem_pointer = 0;
int BATT_SOC = 1000;
uint16_t MSCAL[6];
uint32_t PRES = 0;
uint32_t TEMP = 0;

FATFS FatFs; //Object for FAT storage
FIL fil; //File object
FRESULT fr; //FatFS return code
FILINFO fno;

int16_t main(void) {
#ifdef SD_CARD
    uint32_t temp = 0;
#endif
    int i = 0;

    aerrno = ERR_OK; /*global error var*/

    
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    //Set all ports to output to minimise current leakage
    //Any updated ports will be changed after this so no biggy
    //Drive them low?
    init_ports();
    TRISDbits.TRISD11 = 1;
    
    //setup remappable peripherals
    __builtin_write_OSCCONL(OSCCON & ~0x40);
            //SPI
    SPI1CLK = 8; //Good luck Matches altium 
    SPI1MOSI = 7;
    _SDI1R = 24;
            //UART
    UART_TX = 3; //D10
    //UART_RX = 12; //d11 (RP12)
    //_U1RXR = 11; //D11
    RPINR18bits.U1RXR = 12;
    
    __builtin_write_OSCCONL(OSCCON | 0x40); 

    spi1Init(0);
    
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

    TRISDbits.TRISD8 = 0; //SD_CTL
    TRISDbits.TRISD0 =  0; //SD_CS

    LATDbits.LATD8 = 1; //drive high SD OFF
    LATDbits.LATD0 = 1;

    TimerInit();

    MS5637_ON;
    delay_us_3(1000);
    MS5637_READ_CALIBRATION(MSCAL);
    MS5637_OFF;

    RTC_ALARMSET(1); //every second, see pg 280 "ALCFGRPT" register. NOT 1:1

    for(i=0;i<10;i++) { //Flash LED to say we're on
        LED_ON;
        delay(1000);
        LED_OFF;
        delay(1000);
    }

#ifdef SD_CARD
    TIMER_START;
    SD_ON;	
    disk_initialize(0);
    fr = f_mount(&FatFs, "", 1);
    if (fr !=  FR_OK) {
        //Hold on LED, refuse to start
        while(1) { LED_ON; }
    }
    fr = f_open(&fil, "a.bin", FA_WRITE | FA_OPEN_EXISTING);
    if (fr == FR_NO_FILE) {
        fr = f_open(&fil, "a.bin", FA_WRITE | FA_CREATE_NEW);
    }
    fr = f_write(&fil, (void*)MSCAL, (UINT)sizeof(MSCAL), (UINT*)&temp);
    f_close(&fil);
    TIMER_END;
    SD_OFF;
#else
    UART1Init(6); //Gives 142000 Baud rate. Clock at approx 20Mhz (10mHz FCY)
#endif
 /*   while(1) {
        UART1PutChar('F');
        if(UART1GetChar()) {
            UART1PutChar('G');
            UART1PutChar('\n');
        }
    }*/
    
    while(1) {
        MS5637_OFF;
        MPU9150_OFF;
        LATDbits.LATD7 = 0;

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
            Nop();
            Nop();
            //Think harder about what happens near the limits of size
            if(meas_index + mem_pointer < EXT_MEM_SIZE) {
                //write to device 1
                for(i = 0; i < meas_index; i++) {
                    EXT_MEM_write_buffer(0, mem_pointer, sizeof(struct MEASUREMENT),
                            (uint8_t*)&(measure[i]));
                    mem_pointer += sizeof(struct MEASUREMENT);
                }
            } else if(meas_index*sizeof(struct MEASUREMENT) + mem_pointer < 2*EXT_MEM_SIZE) {
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
#ifdef SD_CARD                      
                        SD_CARD_WRITE_STRUCT(measure);
#endif
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
#ifdef SD_CARD
                    SD_CARD_WRITE_STRUCT(measure);
#endif
                }
            }
            UART1Init(6);
            delay(100);
            UART_SEND_PACKET();
            UART1PutChar('\n');
            delay(100);
            meas_index = 0;
            RTC_ALARMSET(1);
        }

        
        //ENTER_SLEEP;
        ENTER_LV_SLEEP;
    }
}

int UART_SEND_PACKET(void) {
    int  i, k;
    int16_t f;
    //This dumps all memory

    for(i = 0; i < MAX_MEASURE; i++) {
        for(k = 0; k < sizeof(struct MEASUREMENT)/2; k++) {
            Nop();
            Nop();
            f = *( ((int*)&measure[i] + k) );
            UART1PutChar( (f)&0xFF);
            UART1PutChar( (f >> 8)&0xFF );
        }
        UART1PutChar('\n');
    }
    //UART1PutChar('T');
    //mem_pointer
    //meas_index
    return (0);
}

int SD_CARD_WRITE_STRUCT(struct MEASUREMENT *measure) {
    int i, temp;
    
    TIMER_START;
    SD_ON;
    disk_initialize(0);
    fr = f_mount(&FatFs, "", 1);
    if (fr !=  FR_OK) {
        return 1;
    }
    fr = f_open(&fil, "a.bin", FA_WRITE | FA_OPEN_EXISTING);
    if (fr == FR_NO_FILE) {
        fr = f_open(&fil, "a.bin", FA_WRITE | FA_CREATE_NEW);
    }
    f_lseek(&fil, f_size(&fil)); //end of file
    
    if(fr != FR_OK) {
        return 1;
    }

    for (i = 0; i < MAX_MEASURE; i++) {
        fr = f_write(&fil, (void*)&measure[i],
                (UINT)sizeof(struct MEASUREMENT),
                (UINT*)&temp);
        if(fr != FR_OK) {
            return 1;
        }
    }
    f_close(&fil);
    TIMER_END;
    SD_OFF;
    return 0;
}

//Set up Timer, target 1kHz interrupts
void TimerInit(void)
{
   //Want 1kHz
   PR1 = 0x3C00;
   IPC0bits.T1IP = 5;	 //set interrupt priority
   T1CON = 0b0000000000000000;	//turn off the timer
   IFS0bits.T1IF = 0;	 //reset interrupt flag
   IEC0bits.T1IE = 1;	 //turn on the timer1 interrupt
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

void
init_ports(void)
{
    //Just set every single port as output
    //The input buffer really has trouble with floating pins
    TRISB = 0x0000;
    TRISC = 0x0000;
    TRISD = 0x0000;
    TRISE = 0x0000;
    TRISF = 0x0000;
    TRISG = 0x0000;
}