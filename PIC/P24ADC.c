
#include "system.h"
#include "P24ADC.h"
#include "adc.h"

void ADC_init(void)
{

    TRISBbits.TRISB2 = 0b1;  // set pin as input (Analog2) Light sensor
    ANSBbits.ANSB2 = 0b1;  // set pin as analog
    
    AD1CON1 = (0b1 << 13) |  (0b00 << 8) ; // set v+ reference to Vdd
                                           // set v- reference to GND
                                           // set negative input to GND
                                           // right justify the output
    AD1CON2 = 0; //(0b1 << 11); // Set results to go to relevant channels
    AD1CHS = 2;
    AD1CON3 = 0x1F02;
    
}

//main process function
int ADCSample(void)
{
    int retval;
    AD1CON1bits.ADON     = 0b1;  // turn on the ADC
    
    AD1CON1bits.SAMP = 1;
    delay_us_3(5000); //Sample and hold ~ 4ms
    AD1CON1bits.SAMP = 0;

    while (!AD1CON1bits.DONE);
    retval = ADC1BUF0;

    AD1CON1bits.ADON     = 0;
    return retval;
}