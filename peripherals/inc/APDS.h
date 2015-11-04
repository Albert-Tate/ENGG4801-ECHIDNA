/**********************************************************
 *  File: APDS.h
 *  Description: Header file for light sensor, uses ADC
 *  Author: Albert Tate
 *  Date: 17/05/15
 **********************************************************/
 
#ifndef __APDS_H__
#define __APDS_H__

    #define APDS_ON command_set_gpio
    #define APDS_OFF command_reset_gpio
    uint16_t APDS_read(void);

#endif