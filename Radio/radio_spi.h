


#ifndef __RADIO_SPI_H

	#define 	__RADIO_SPI_H

	#include "spi.h"

	extern void vSpiMasterInit(void);
	extern unsigned char bSpiWriteByte(unsigned char spi_adr, unsigned char spi_dat);
	extern unsigned char bSpiReadByte(unsigned char spi_adr);
	extern void vSpiBurstWrite(unsigned char spi_adr, unsigned char spi_dat[], unsigned char spi_length);
	extern void vSpiBurstRead(unsigned char spi_adr, unsigned char spi_dat[], unsigned char spi_length);

#endif


