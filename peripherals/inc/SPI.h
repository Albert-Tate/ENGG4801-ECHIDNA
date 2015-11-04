/*
Engscope.com
JL
Created		Jul 1, 2010
Modified	Jul 1, 2010
headers for spi device drivers
*/

enum SPIPorts{
	SPIPORT1 = 1
	#if defined(spi_v1_1) || defined (spi_v1_3)
	, SPIPORT2
	#endif
	#if defined (spi_v1_3)
	, SPIPORT3
	#endif
};

//with selectable ports

unsigned char spiWrite( unsigned port, unsigned char i);

unsigned char spi1Write( unsigned char i );
void spi1Init(unsigned int prescale);

//spi port 2
#if defined(i2c_v1_2) || defined (i2c_v1_3)
unsigned char spi2Write( unsigned char i );
void spi2Init(unsigned int prescale);
#endif

//spi port 3
#if defined (i2c_v1_3)
unsigned char spi3Write( unsigned char i );
void spi3Init(unsigned int prescale);
#endif