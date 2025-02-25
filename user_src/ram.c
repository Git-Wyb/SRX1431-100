/***********************************************************************/
/*  FILE        :ram.c                                                 */
/*  DATE        :Mar, 2014                                             */
/*  Programmer	:xiang 'R                                              */
/*  CPU TYPE    :STM8L151G6     Crystal: 16M HSI                       */
/*  DESCRIPTION :                                                      */
/*  Mark        :ver 1.0                                               */
/***********************************************************************/
#include  <iostm8l151g4.h>				// CPU型号
//#include "stm8l15x.h"
#include "Pin_define.h"		// 管脚定义
//#include "ram.h"
#include "initial.h"

volatile union{
	unsigned char BYTE;
	struct {
		unsigned char	Bit0:	1;
		unsigned char	Bit1:	1;
		unsigned char	Bit2:	1;
		unsigned char	Bit3:	1;
		unsigned char	Bit4:	1;
		unsigned char	Bit5:	1;
		unsigned char	Bit6:	1;
		unsigned char	Bit7:	1;
	}BIT;
}FLAG0;

volatile union{
	unsigned char BYTE;
	struct {
		unsigned char	Bit0:	1;
		unsigned char	Bit1:	1;
		unsigned char	Bit2:	1;
		unsigned char	Bit3:	1;
		unsigned char	Bit4:	1;
		unsigned char	Bit5:	1;
		unsigned char	Bit6:	1;
		unsigned char	Bit7:	1;
	}BIT;
}FLAG1;

volatile union{
	unsigned char BYTE;
	struct {
		unsigned char	Bit0:	1;
		unsigned char	Bit1:	1;
		unsigned char	Bit2:	1;
		unsigned char	Bit3:	1;
		unsigned char	Bit4:	1;
		unsigned char	Bit5:	1;
		unsigned char	Bit6:	1;
		unsigned char	Bit7:	1;
	}BIT;
}FLAG_test;

volatile union{
	unsigned char BYTE;
	struct {
		unsigned char	Bit0:	1;
		unsigned char	Bit1:	1;
		unsigned char	Bit2:	1;
		unsigned char	Bit3:	1;
		unsigned char	Bit4:	1;
		unsigned char	Bit5:	1;
		unsigned char	Bit6:	1;
		unsigned char	Bit7:	1;
	}BIT;
}FLAG_test1;

volatile union{
	unsigned char BYTE;
	struct {
		unsigned char	Bit0:	1;
		unsigned char	Bit1:	1;
		unsigned char	Bit2:	1;
		unsigned char	Bit3:	1;
		unsigned char	Bit4:	1;
		unsigned char	Bit5:	1;
		unsigned char	Bit6:	1;
		unsigned char	Bit7:	1;
	}BIT;
}Mark0 = {0};

char rssi = 0;
short RAM_RSSI_AVG = 0;
long RAM_RSSI_SUM = 0;
UINT8 RSSI_Read_Counter = 0;

UINT8 SIO_cnt;
UINT8 SIO_buff[16];
UINT8 SIO_DATA[16];
UINT8 Tx_Rx_mode=0;
//ADF70XX_REG_T ROM_adf7012_value[16];
/*const ADF70XX_REG_T Default_adf7012_value[16]={0x00000000,0x0176D051,0x00000000,0x00000000,
                                               0x00000000,0x00000000,0x00000000,0x00000000,
                                               0x00000000,0x00000000,0x00000000,0x00000000,
                                               0x00000000,0x00000000,0x00000000,0x00000000,
                                               };*/
UINT8 Time_APP_blank_TX = 0;
UINT8  TIME_10ms=0;
UINT16 TIME_auto_useful = 0;
UINT8 FREQ_auto_useful = 0;
UINT8 FREQ_auto_useful_count = 0;
UINT16 FREQ_auto_useful_continuous = 0;
UINT16  TIMER1s=0;
UINT16  TIMER300ms=0;
UINT16  TIMER18ms=0;
UINT8   TIMER250ms_STOP=0;
UINT16  TIME_auto_out=0;
UINT16  TIME_auto_close=0;
UINT16 time_3sec=0;
UINT32 ID_Receiver_DATA[256] = {0};//写入EEPROM ID的数据
UINT16 ID_DATA_PCS=0;
UINT8 rxphase=0;
UINT32 DATA_Packet_Syn=0;     //A部
UINT16 DATA_Packet_Head=0;    //B部
UINT32 DATA_Packet_Code[6]={0};   //C部
UINT8  DATA_Packet_Code_g=0;
UINT8  DATA_Packet_Code_i=0;
UINT32 DATA_Packet_ID=0;
UINT8  DATA_Packet_Control=0;
UINT8  DATA_Packet_Contro_buf=0;   //2015.3.24修正
UINT32 ID_Receiver_Login=0;
UINT8 TIME_EMC=0;   //静电测试

UINT16 INquiry=0;
UINT16 TIME_Receiver_Login_restrict=0;
UINT8  COUNT_Receiver_Login=0;
UINT16 TIME_Receiver_Login=0;
UINT16 TIME_Login_EXIT_rest=0;
UINT16 TIME_Receiver_Login_led=0;

UINT8 TIME_OUT_OPEN_CLOSE=0;
UINT16 TIME_Receiver_LED_OUT=0;
UINT16 TIME_Login_EXIT_Button=0;

UINT16 Manual_override_TIMER=0;

UINT16 RAM_rssi_SUM=0;
UINT8 RAM_rssi_CNT=0;
UINT8 RAM_rssi_AVG=0;

UINT16 X_COUNT = 0;
UINT16 X_ERR = 0; //记录错误的个敿
UINT16 X_ERRTimer = 0;

UINT16 time_Login_exit_256=0;

//UINT16 TIME_Fine_Calibration=0;   //窄带下中频滤波器100KHz精校

//ADF70XX_REG_T RSSI_value_buf;  //RSSI 测试

UINT32 Freq_Value = 426075000ul;

UINT16 Time_Nms = 0;
UINT16 Time_Tx_Out = 0;

UINT8 CONST_TXPACKET_DATA_20000AF0[28] = {
	0X95, 0X55, 0X55, 0X55,
	0X55, 0X55, 0X56, 0X55,
	0X95, 0X55, 0X56, 0X55,
	0X95, 0X55, 0X55, 0X55,
	0X95, 0X55, 0X55, 0X55,
	0X95, 0X55, 0X55, 0X55,
	0X95, 0X55, 0X55, 0X55};

void delay_nms(UINT16 nms)
{
    Time_Nms = nms;
    while(Time_Nms)
    {
        ClearWDT();
    }
}

void delay1ms(UINT16 u16Cnt)
{
    delay_nms(u16Cnt);
}

void delay_nus(UINT8 nus)
{
    UINT8 i = 0;
    while(nus)
    {
        for(i=0; i<5; i++)
        {
            __asm("nop");//163.9ns
        }
        nus--;
        ClearWDT();
    }
}

void delay10us(UINT8 nus)
{
    UINT8 i = 0;
    for(i=0; i<nus; i++)
    {
        delay_nus(10);
    }
}
