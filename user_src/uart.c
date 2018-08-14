/***********************************************************************/
/*  FILE        :Uart.c                                                */
/*  DATE        :Mar, 2014                                             */
/*  Programmer	:xiang 'R                                              */
/*  CPU TYPE    :STM8L151G6     Crystal: 16M HSI                       */
/*  DESCRIPTION :                                                      */
/*  Mark        :ver 1.0                                               */
/***********************************************************************/
#include  <iostm8l151g6.h>				// CPU�ͺ� 
#include "Pin_define.h"		// �ܽŶ���
#include "initial.h"		// ��ʼ��  Ԥ����
#include "ram.h"		// RAM����

#define	TXD1_enable	(USART1_CR2 = 0x08)		// ������	
#define RXD1_enable	(USART1_CR2 = 0x24)		// ������ռ����ж�	
//********************************************
void UART1_INIT(void){						// 
	USART1_CR1 = 0;							// 1����ʼλ,8������λ 
	USART1_CR3 = 0;							// 1��ֹͣλ 
	USART1_CR4 = 0;
	USART1_CR5 = 0x08;						// ��˫��ģʽ
	USART1_BRR2 = 0x03;						// ���ò�����9600
	USART1_BRR1 = 0x68;						// 3.6864M/9600 = 0x180
	                                                                //16.00M/9600 = 0x683
	USART1_CR2 = 0x08;	// ������
        //USART1_CR2 = 0x24;

} 
//--------------------------------------------
void UART1_RX_RXNE(void){		// RXD�жϷ������ 
//	unsigned char dat;
//	dat = USART1_DR;							// ��������
//	
//	if (dat=='(')  SIO_cnt = 0;
//	SIO_buff[SIO_cnt] = dat;
//	SIO_cnt = (SIO_cnt + 1) & 0x1F;
//	if (dat==')'){
//		for (dat=0;dat<SIO_cnt;dat++) {
//			SIO_DATA[dat] = SIO_buff[dat];
//		}
//		BIT_SIO = 1;						// ��־
//		SIO_TOT = 20;	
//	}
	
} 


//--------------------------------------------
void Send_char(unsigned char ch){			// �����ַ�
	//TXD1_enable;							// ������	
	while(!USART1_SR_TXE);
	USART1_DR = ch;							// ����
	while(!USART1_SR_TC);					// �ȴ���ɷ���
	//RXD1_enable;							// ������ռ����ж�	
}
//--------------------------------------------
void Send_String(unsigned char *string){	// �����ַ���
	unsigned char i=0;
	TXD1_enable;							// ������	
	while (string[i]){
		while(!USART1_SR_TXE);				// ��鷢��OK
		USART1_DR = string[i];				// ����
		i++;
	}
	while(!USART1_SR_TC);					// �ȴ���ɷ���
	RXD1_enable;							// ������ռ����ж�	
//	BIT_SIO = 0;							// ��־	
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