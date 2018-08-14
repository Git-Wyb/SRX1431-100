/***********************************************************************/
/*  FILE        :Uart.c                                                */
/*  DATE        :Mar, 2014                                             */
/*  Programmer	:xiang 'R                                              */
/*  CPU TYPE    :STM8L151G6     Crystal: 16M HSI                       */
/*  DESCRIPTION :                                                      */
/*  Mark        :ver 1.0                                               */
/***********************************************************************/
#include  <iostm8l151g6.h>				// CPU型号 
#include "Pin_define.h"		// 管脚定义
#include "initial.h"		// 初始化  预定义
#include "ram.h"		// RAM定义

#define	TXD1_enable	(USART1_CR2 = 0x08)		// 允许发送	
#define RXD1_enable	(USART1_CR2 = 0x24)		// 允许接收及其中断	
//********************************************
void UART1_INIT(void){						// 
	USART1_CR1 = 0;							// 1个起始位,8个数据位 
	USART1_CR3 = 0;							// 1个停止位 
	USART1_CR4 = 0;
	USART1_CR5 = 0x08;						// 半双工模式
	USART1_BRR2 = 0x03;						// 设置波特率9600
	USART1_BRR1 = 0x68;						// 3.6864M/9600 = 0x180
	                                                                //16.00M/9600 = 0x683
	USART1_CR2 = 0x08;	// 允许发送
        //USART1_CR2 = 0x24;

} 
//--------------------------------------------
void UART1_RX_RXNE(void){		// RXD中断服务程序 
//	unsigned char dat;
//	dat = USART1_DR;							// 接收数据
//	
//	if (dat=='(')  SIO_cnt = 0;
//	SIO_buff[SIO_cnt] = dat;
//	SIO_cnt = (SIO_cnt + 1) & 0x1F;
//	if (dat==')'){
//		for (dat=0;dat<SIO_cnt;dat++) {
//			SIO_DATA[dat] = SIO_buff[dat];
//		}
//		BIT_SIO = 1;						// 标志
//		SIO_TOT = 20;	
//	}
	
} 


//--------------------------------------------
void Send_char(unsigned char ch){			// 发送字符
	//TXD1_enable;							// 允许发送	
	while(!USART1_SR_TXE);
	USART1_DR = ch;							// 发送
	while(!USART1_SR_TC);					// 等待完成发送
	//RXD1_enable;							// 允许接收及其中断	
}
//--------------------------------------------
void Send_String(unsigned char *string){	// 发送字符串
	unsigned char i=0;
	TXD1_enable;							// 允许发送	
	while (string[i]){
		while(!USART1_SR_TXE);				// 检查发送OK
		USART1_DR = string[i];				// 发送
		i++;
	}
	while(!USART1_SR_TC);					// 等待完成发送
	RXD1_enable;							// 允许接收及其中断	
//	BIT_SIO = 0;							// 标志	
}


/***********************************************************************/
unsigned char asc_hex(unsigned char asc)	// HEX
{
	unsigned char i;
	if (asc < 0x3A) i = asc & 0x0F; 
	else i = asc - 0x37;
	return i;
}

unsigned char hex_asc(unsigned char hex)
{
	unsigned char i;
	hex = hex & 0x0F;
	if (hex < 0x0A) i = hex | 0x30;
	else i = hex + 0x37;
	return i;
}

unsigned char asc_hex_2(unsigned char asc1,unsigned char asc0)
{                                    
	unsigned char i; 
	i = (asc_hex(asc1) << 4) + (asc_hex(asc0) & 0x0F);
	return i;
} 