/*
Engscope
UART
April 16, 2008
Author: JL
*/

//Initiation
extern void UART1Init(int BAUDRATEREG1);

//UART transmit function
extern void  UART1PutChar(char Ch);

//UART receive function
extern char UART1GetChar();