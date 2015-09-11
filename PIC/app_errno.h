/**********************************************************
 *  File: app_errno.h
 *  Description: Driver for I2C
 *  Author: Albert Tate
 *  Date: 32/08/15
 **********************************************************/


#ifndef __APP_ERRNO_H
#define __APP_ERRNO_H

extern int aerrno;

#define ERR_OK 0
#define I2CSENDTIMEOUT 1
#define I2CRECTIMEOUT 2
#define I2CSTART 3 /*Failure on Start condition*/
#define I2CSTOP 4 /*Failure on Stop condition*/
#define I2CACK 5 /*Failure on ACK*/

#endif