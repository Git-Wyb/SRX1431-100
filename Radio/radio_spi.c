#include "radio_spi.h"	

#define		SPI_SPEED_SEL		5				// 0 --> 6

#define		SPI_DUMMY_VALUE		0x00

/******************************
**Name: vSpiMasterInit
**Func: SPI Master
**Input: None
*Output: None
********************************/
void vSpiMasterInit(void)
{
	stc_spi_cfg_t  SpiInitStruct;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi0,TRUE);

    SpiInitStruct.enSpiMode = SpiMskMaster;   
	
	switch(SPI_SPEED_SEL)
		{															//                  @24M    @32M   @40M
		case 6:  SpiInitStruct.enPclkDiv = SpiClkMskDiv2;  break;   //bit rate divide	 12M     16M    20M 
		case 5:  SpiInitStruct.enPclkDiv = SpiClkMskDiv4;  break;   //bit rate divide	  6M      8M    10M
		case 4:  SpiInitStruct.enPclkDiv = SpiClkMskDiv8;  break;	//bit rate divide	  3M      4M	 5M		
		case 3:                                                     
		default: SpiInitStruct.enPclkDiv = SpiClkMskDiv16; break;  	//bit rate divide	1.5M      2M   2.5M	  		
		case 2:  SpiInitStruct.enPclkDiv = SpiClkMskDiv32; break;  	//bit rate divide	750k      1M   1.25M
		case 1:  SpiInitStruct.enPclkDiv = SpiClkMskDiv64; break;	//bit rate divide	375k    500k    625k	
		case 0:  SpiInitStruct.enPclkDiv = SpiClkMskDiv128;break;	//bit rate divide	188k    250k    313k
		}
	
	SpiInitStruct.enCPHA    = SpiMskCphafirst;		//first edge 
    SpiInitStruct.enCPOL    = SpiMskcpollow;  		//SCLK active for high
    Spi_Init(M0P_SPI0, &SpiInitStruct);

}	


unsigned char bSpiWriteByte(unsigned char spi_adr, unsigned char spi_dat) 
{
	spi_adr &= 0x7F;
	
	Spi_SetCS(M0P_SPI0, FALSE);
    Spi_SendData(M0P_SPI0, spi_adr);
	while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);
    Spi_SendData(M0P_SPI0, spi_dat);
	while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);
	Spi_SetCS(M0P_SPI0, TRUE);	
	return(M0P_SPI0->DATA);
}	

unsigned char bSpiReadByte(unsigned char spi_adr)
{
	spi_adr |= 0x80;
	
	Spi_SetCS(M0P_SPI0, FALSE);
    Spi_SendData(M0P_SPI0, spi_adr);
	while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);
    Spi_SendData(M0P_SPI0, SPI_DUMMY_VALUE);
	while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);
	Spi_SetCS(M0P_SPI0, TRUE);	
	return(M0P_SPI0->DATA);	
}

void vSpiBurstWrite(unsigned char spi_adr, unsigned char spi_dat[], unsigned char spi_length)
{
	byte i;
	
	spi_adr &= 0x7F;
	
	Spi_SetCS(M0P_SPI0, FALSE);
    Spi_SendData(M0P_SPI0, spi_adr);
	while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);
	
	for(i=0; i<spi_length; i++)
		{
		Spi_SendData(M0P_SPI0, spi_dat[i]);
		while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);	
		}
	
	Spi_SetCS(M0P_SPI0, TRUE);	
	return;
}	

void vSpiBurstRead(unsigned char spi_adr, unsigned char spi_dat[], unsigned char spi_length)
{
	unsigned char i;
	
	spi_adr |= 0x80;

	Spi_SetCS(M0P_SPI0, FALSE);
    Spi_SendData(M0P_SPI0, spi_adr);
	while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);

	for(i=0; i<spi_length; i++)
		{
		Spi_SendData(M0P_SPI0, SPI_DUMMY_VALUE);
		while(Spi_GetStatus(M0P_SPI0, SpiBusy) == TRUE);	
		spi_dat[i] = M0P_SPI0->DATA;	
		}

	Spi_SetCS(M0P_SPI0, TRUE);	
	return;
}	

