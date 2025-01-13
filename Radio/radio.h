

#ifndef __RADIO_H

	#define 	__RADIO_H

	#include "radio_phy.h"
	#include "radio_mac.h"
	#include "CMT2310A_def.h"
	#include "CMT2310A_reg.h"

	#define	 	UHF_LEN				12					//
    #define SPI_REV_BUFF_LONG 40

    typedef enum
    {
        DEV_2_0K = 1,
        DEV_2_4K = 2,
    }DEVSET_ENUM;

    typedef enum
    {
        RATE_1_2K = 1,
        RATE_2_4K = 2,
        RATE_4_8K = 3,
        RATE_9_6K = 4
    }DataRate_ENUM;

    extern u8 radio_tx_buf[UHF_LEN];
    extern u8 radio_rx_buf[UHF_LEN];
    extern u8 SPI_RECEIVE_BUFF[SPI_REV_BUFF_LONG];
	extern uint32_t g_chip_id;
    extern CMT2310A_CFG	g_radio;
    extern u32 SPI_Receive_DataForC[7];
    extern u8 APP_TX_freq;
    extern u8 Radio_Date_Type;

	extern unsigned char g_cmt2310a_page0[CMT2310A_PAGE0_SIZE];
	extern unsigned char g_cmt2310a_page1[CMT2310A_PAGE1_SIZE];
	extern const unsigned char g_cmt2310a_page2[CMT2310A_PAGE2_SIZE];
    extern Wireless_Body Struct_DATA_Packet_Contro,Struct_DATA_Packet_Contro_buf;
    extern Wireless_Body Uart_Struct_DATA_Packet_Contro,Last_Uart_Struct_DATA_Packet_Contro;

	void vRadioInit(u8 mode);
	extern void vRadioClearInterrupt(void);
	extern void vRadioReadAllStatus(void);
	extern void vRadioCmpReg(byte const wr_ptr[], byte rd_ptr[], byte cmp_ptr[], byte length);
	extern void vRadioGoTxInit(void);
	extern void vRadioGoRxInit(void);
    void RF_CMT2310A_Init(u8 mode);
    void CMT2310A_Data_Mode(u8 mode);
    void CMT2310A_Set_Freq(u32 freq,DEVSET_ENUM dev);
    void CMT2300A_TxData(u8 *txbuff,u8 len);
    void CMT2300A_RxDone(void);
    void CMT2300A_TxDone(void);
    u8 CMT2310A_ReadReg(u8 page,u8 addr);
    void CMT2310A_SetRx(void);
    void CMT2310A_SetTx(void);
    void RX_ANALYSIS(void);
    void CMT2300A_ReadData(u8 *rxbuff,u8 len);
    void TX_DataLoad(u32 IDCache, u8 CtrCmd, u8 *Packet);
    void APP_TX_PACKET(void);
    void CMT2310A_Freq_Select(u32 freq);
    u8 CMT2310A_Get_RSSI(void);
    void CMT2310A_DataRate_Select(DataRate_ENUM rate);
    void TX_DataLoad_HighSpeed(u32 IDCache, Wireless_Body CtrCmd, u8 *Packet);
    void CMT2310A_Freq_Scanning(void);
    void CMT2310A_Change_Channel(void);
    void CMT2310A_Frequency_Set(u32 freq,u8 radio_type);

#endif




