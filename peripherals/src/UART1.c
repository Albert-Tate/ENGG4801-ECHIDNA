/*
Engscope
UART
April 16, 2008
Author: JL
*/

#include "uart.h"
#include "../inc/UART1.h"

//Initiation function
//parameter BAUDRATEREG1 determines baud speed
void UART1Init(int BAUDRATEREG1)
{
   //Set up registers
   U1BRG = BAUDRATEREG1;	//set baud speed
   U1MODE	=	0x8000;	 //turn on module
   U1STA	=	0x8400;	 //set interrupts
   //reset RX interrupt flag
   IFS0bits.U1RXIF = 0;
}

//UART transmit function, parameter Ch is the character to send
void UART1PutChar(char Ch)
{
   //transmit ONLY if TX buffer is empty
   while(U1STAbits.UTXBF == 1);
   U1TXREG = Ch;
}

//UART receive function, returns the value received.
char UART1GetChar(void)
{
   char Temp;
   //wait for buffer to fill up, wait for interrupt
   if(IFS0bits.U1RXIF == 0) {
       return 0;
   }
   Temp = U1RXREG;
   //reset interrupt
   IFS0bits.U1RXIF = 0;
   //return my received byte
   return Temp;
}