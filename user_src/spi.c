/**
****************************************************************************
* @Warning :Without permission from the author,Not for commercial use
* @File    :spi.c
* @Author  :Xiaowine
* @date    :2017/4/13
* @version :V 1.0.0
*************************************************
* @brief   :
****************************************************************************
**/

#include "spi.h"



//-----------------------------------------------CMT2310A---------------------------------------------------------

void spi_delay(u16 n) //1: 1.69us; 2: 2.24us; 4: 3.19us; 10: 6.2us
{
     while(n--);
}

/* SPI相关IO口初始化,MISO->PB7;MOSI->PB6;SCK->PB5;CS->PB4;FCSB->PD0 */
void SPI1_Gpio_Config(void)
{
    SPI_CS_DDR = 1;     /* CS 设置数据方向寄存器 1为输出，0为输入--查看STM8寄存器.pdf P87 */
    SPI_CS_CR1 = 1;     /* 设置推挽输出--查看STM8寄存器RM0031.pdf 10.9 */
    SPI_CS_CR2 = 1;     /* 设置输出频率 1为10M，0为2M--查看STM8寄存器.pdf P89 */

    SPI1_SCK_DDR = 1;   /* SCK output */
    SPI1_SCK_CR1 = 1;
    SPI1_SCK_CR2 = 1;

    SPI1_MOSI_DDR = 1;  /* MOSI output */
    SPI1_MOSI_CR1 = 1;
    SPI1_MOSI_CR2 = 1;

    SPI1_MISO_DDR = 0;  /* MISO input */
    SPI1_MISO_CR1 = 1;  /* in put with pull-up */
    SPI1_MISO_CR2 = 0;
}

/* 初始化SPI1 */
void SPI_Config_Init(void)
{
    SPI1_Gpio_Config();

    CLK_PCKENR1 |= 0x10;    /* 开启SPI1时钟 */
    SPI1_CR1_SPE = 0;      //禁止SPI1
    SPI1_CR1_LSBFIRST = 0; //先发送MSB

    SPI1_CR1_BR = 0;     //0:fSYSCLK/2 = 8MHz; 3:fSYSCLK/16 = 1MHz

    SPI1_CR1_MSTR = 1;     //Master configuration 设置为主模式
    SPI1_CR1_CPOL = 0;     //0: SCK to 1 when idle		空闲状态时SCK为低电平									(MISO和MOSI在CLK的上升沿载入，下降沿取样)
    SPI1_CR1_CPHA = 0;     //0: The first clock transition is the second data capture edge (MISO和MOSI在CLK的上升沿载入，下降沿取样)

    SPI1_CR2_BDM = 0;

    SPI1_CR2_CRCEN = 0;  //0: CRC calculation disabled
    SPI1_CR2_RXONLY = 0; //0: Full duplex (Transmit and receive)
    SPI1_CR2_SSM = 1;    //0: Software slave management disabled
    SPI1_CR2_SSI = 1;    //1: Master mode

    SPI1_ICR_TXIE = 0;    //TXE interrupt masked
    SPI1_ICR_RXIE = 0;    //RXNE interrupt masked
    SPI1_ICR_ERRIE = 0;   //Error interrupt is masked
    SPI1_ICR_WKIE = 0;    //Wakeup interrupt masked
    SPI1_ICR_TXDMAEN = 0; //Tx buffer DMA disabled
    SPI1_ICR_RXDMAEN = 0; //Rx buffer DMA disabled

    SPI1_CR1_SPE = 1; //使能SPI1

    SPI1_CS_H;
}


/*
函数功能：SPI1发送、接收数据
函数形参：要发送的数据
*/
u8 SPI1_SendRecv_Data(u8 byte)
{
    u8 dat;
    dat = SPI1_DR;
    while (!(SPI1_SR & 0x02));/* 等待发送寄存器为空 */

    SPI1_DR = byte; /* 将发送的数据写到数据寄存器 */
    __asm("nop");//163.9ns

    while (!(SPI1_SR & 0x01))   //8us结束/* 等待接受寄存器满 */
    {
        ;
    }

    dat = SPI1_DR;
    return dat;
}

u8 CMT2310A_Write_Reg(u8 addr,u8 dat)
{
    u8 data = 0;

    addr &= 0x7F;

    spi_delay(1);
    SPI1_CS_L;
    spi_delay(1);

    SPI1_SendRecv_Data(addr);
    data = SPI1_SendRecv_Data(dat);

    spi_delay(1);
    SPI1_CS_H;
    spi_delay(1);

    return data;
}

u8 CMT2310A_Read_Reg(u8 addr)
{
    u8 read = 0;

    addr |= 0x80;

    spi_delay(1);
    SPI1_CS_L;
    spi_delay(1);

    SPI1_SendRecv_Data(addr);       /* 写地址 */
    read = SPI1_SendRecv_Data(0x00);/* 读数据 */

    spi_delay(1);
    SPI1_CS_H;
    spi_delay(1);

    return read;
}

unsigned char bSpiWriteByte(unsigned char spi_adr, unsigned char spi_dat)
{
    return CMT2310A_Write_Reg(spi_adr,spi_dat);
}
unsigned char bSpiReadByte(unsigned char spi_adr)
{
    return CMT2310A_Read_Reg(spi_adr);
}

void vSpiBurstWrite(unsigned char spi_adr, unsigned char spi_dat[], unsigned char spi_length)
{
    u8 i = 0;

    spi_adr &= 0x7F;

    SPI1_CS_L;
    SPI1_SendRecv_Data(spi_adr);
    for(i=0; i<spi_length; i++)
    {
        SPI1_SendRecv_Data(spi_dat[i]);
    }
    SPI1_CS_H;
}
void vSpiBurstRead(unsigned char spi_adr, unsigned char spi_dat[], unsigned char spi_length)
{
    u8 i = 0;

    spi_adr |= 0x80;

    SPI1_CS_L;
    SPI1_SendRecv_Data(spi_adr);
    for(i=0; i<spi_length; i++)
    {
        spi_dat[i] = SPI1_SendRecv_Data(0x00);
    }
    SPI1_CS_H;
}
