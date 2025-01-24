/***********************************************************************/
/*  FILE        :ram.h                                                 */
/*  DATE        :Mar, 2014                                             */
/*  Programmer	:xiang 'R                                              */
/*  CPU TYPE    :STM8L151G6     Crystal: 16M HSI                       */
/*  DESCRIPTION :                                                      */
/*  Mark        :ver 1.0                                               */
/***********************************************************************/
#include "initial.h"		// 初始化  预定义

extern volatile union{
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
	//************************************************
	#define 	FLAG0_BYTE		FLAG0.BYTE
	//------------------------------------------------
        #define		FLAG_Receiver_IDCheck	FLAG0.BIT.Bit0
        #define		FLAG_Signal_DATA_OK	FLAG0.BIT.Bit1
        #define		FLAG_APP_RX		FLAG0.BIT.Bit2
        #define		FLAG_IDCheck_OK		FLAG0.BIT.Bit3
        #define		FLAG_ID_Erase_Login	FLAG0.BIT.Bit4
        #define		FLAG_ID_Erase_Login_PCS	FLAG0.BIT.Bit5
        #define		FLAG_ID_Login	        FLAG0.BIT.Bit6
        #define		FLAG_ID_Login_OK	FLAG0.BIT.Bit7
	//************************************************

extern volatile union{
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
	//************************************************
	#define 	FLAG1_BYTE		FLAG1.BYTE
	//------------------------------------------------
        #define		FLAG_Receiver_BEEP	FLAG1.BIT.Bit0
        #define		FLAG_ID_Login_EXIT	FLAG1.BIT.Bit1
        #define		FLAG_ID_Login_OK_bank	FLAG1.BIT.Bit2
        #define		FG_beep_on		FLAG1.BIT.Bit3
        #define		FG_beep_off	        FLAG1.BIT.Bit4
        #define		FG_allow_out	        FLAG1.BIT.Bit5
        #define		FG_NOT_allow_out	FLAG1.BIT.Bit6
        #define		FG_10ms		        FLAG1.BIT.Bit7
	//************************************************

extern volatile union{
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
	//************************************************
	#define 	FLAG_test_BYTE		FLAG_test.BYTE
	//------------------------------------------------
        #define		FLAG_Receiver_Scanning	FLAG_test.BIT.Bit0
        #define		FG_test_tx_1010	        FLAG_test.BIT.Bit1
        #define		X_HIS	                FLAG_test.BIT.Bit2    //历史记录   误码率测试用
        #define		FG_test_tx_on		FLAG_test.BIT.Bit3
        #define		FG_test_tx_off	        FLAG_test.BIT.Bit4
        #define		FG_test_mode	        FLAG_test.BIT.Bit5
        #define		FG_test1	        FLAG_test.BIT.Bit6
        #define		FG_test_rx		FLAG_test.BIT.Bit7
	//************************************************

extern volatile union{
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
	//************************************************
	#define 	FLAG_test1_BYTE		FLAG_test1.BYTE
	//------------------------------------------------
        #define		BIT_SIO          	FLAG_test1.BIT.Bit0
        #define		FG_auto_out	        FLAG_test1.BIT.Bit1
        #define		FG_OUT_OPEN_CLOSE	FLAG_test1.BIT.Bit2    //历史记录   误码率测试用
//        #define		FG_auto_outbz		FLAG_test1.BIT.Bit3
        #define		FG_auto_open_time	        FLAG_test1.BIT.Bit4
        #define		FG_auto_manual_mode	        FLAG_test1.BIT.Bit5      //1=auto,0=manual
        #define		FG_Receiver_LED_RX	FLAG_test1.BIT.Bit6
        #define		FG_First_auto		FLAG_test1.BIT.Bit7
	//************************************************

extern volatile union{
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
}Mark0;
#define Flag_TxDone  Mark0.BIT.Bit0
#define Flag_RxDone  Mark0.BIT.Bit1
#define Flag_TxEn    Mark0.BIT.Bit2
#define FLAG_APP_TX  Mark0.BIT.Bit3
#define Flag_FREQ_Scan  Mark0.BIT.Bit4
#define FLAG_ID_SCX1801_Login  Mark0.BIT.Bit5

extern char rssi;
extern short RAM_RSSI_AVG;
extern long RAM_RSSI_SUM;
extern UINT8 RSSI_Read_Counter;
extern UINT16 Time_APP_blank_TX;
extern UINT8 SIO_cnt;
extern UINT8 SIO_buff[16];
extern UINT8 SIO_DATA[16];
extern UINT8 Tx_Rx_mode;
//extern ADF70XX_REG_T ROM_adf7012_value[16];
//extern const ADF70XX_REG_T Default_adf7012_value[16];

extern UINT8 CONST_TXPACKET_DATA_20000AF0[28];
extern UINT8  TIME_10ms;
extern UINT16 TIME_auto_useful;
extern UINT8 FREQ_auto_useful;
extern UINT8 FREQ_auto_useful_count;
extern UINT16 FREQ_auto_useful_continuous;
extern UINT16  TIMER1s;
extern UINT16  TIMER300ms;
extern UINT16  TIMER18ms;
extern UINT8   TIMER250ms_STOP;
extern UINT16  TIME_auto_out;
extern UINT16  TIME_auto_close;
extern UINT16 time_3sec;
extern UINT32 ID_Receiver_DATA[256];//写入EEPROM ID的数据
extern UINT16 ID_DATA_PCS;
extern UINT8 rxphase;
extern UINT32 DATA_Packet_Syn;     //A部
extern UINT16 DATA_Packet_Head;    //B部
extern UINT32 DATA_Packet_Code[6];   //C部
extern UINT8  DATA_Packet_Code_g;
extern UINT8  DATA_Packet_Code_i;
extern UINT32 DATA_Packet_ID;
extern UINT8  DATA_Packet_Control;
extern UINT8  DATA_Packet_Contro_buf;   //2015.3.24修正
extern UINT32 ID_Receiver_Login;
extern UINT8 TIME_EMC;   //静电测试

extern UINT16 INquiry;
extern UINT16 TIME_Receiver_Login_restrict;
extern UINT8  COUNT_Receiver_Login;
extern UINT16 TIME_Receiver_Login;
extern UINT16 TIME_Login_EXIT_rest;
extern UINT16 TIME_Receiver_Login_led;

extern UINT8 TIME_OUT_OPEN_CLOSE;
extern UINT16 TIME_Receiver_LED_OUT;
extern UINT16 TIME_Login_EXIT_Button;

extern UINT16 Manual_override_TIMER;

extern UINT16 RAM_rssi_SUM;
extern UINT8 RAM_rssi_CNT;
extern UINT8 RAM_rssi_AVG;

extern UINT16 X_COUNT;
extern UINT16 X_ERR; //记录错误的个敿
extern UINT16 X_ERRTimer;

extern UINT16 time_Login_exit_256;
extern UINT32 PROFILE_CH_FREQ_32bit_200002EC;
extern const UINT32 PROFILE_CH1_FREQ_32bit_429HighSpeed;
extern const UINT32 PROFILE_CH2_FREQ_32bit_429HighSpeed;
extern UINT8 Channels;
extern UINT32 ID_SCX1801_DATA;
extern UINT8 Flag_TX_ID_load;

//extern UINT16 TIME_Fine_Calibration;   //窄带下中频滤波器100KHz精校

//extern ADF70XX_REG_T RSSI_value_buf;  //RSSI 测试

extern UINT16 Time_Nms;
extern UINT16 Time_Tx_Out;
extern UINT32 Freq_Value;
void delay_nms(UINT32 nms);
void delay1ms(UINT32 u32Cnt);
void delay_nus(UINT8 nus);
void delay10us(UINT8 nus);
