

#ifndef __RADIO_H

	#define 	__RADIO_H

	#include "radio_phy.h"
	#include "radio_mac.h"
	#include "CMT2310A_def.h"
	#include "CMT2310A_reg.h"

	#define	 	UHF_LEN				12					//

    typedef enum
    {
        DEV_2_0K = 1,
        DEV_2_4K = 2,
    }DEVSET_ENUM;

	extern uint32_t g_chip_id;
    extern CMT2310A_CFG	g_radio;

	extern unsigned char g_cmt2310a_page0[CMT2310A_PAGE0_SIZE];
	extern unsigned char g_cmt2310a_page1[CMT2310A_PAGE1_SIZE];
	extern const unsigned char g_cmt2310a_page2[CMT2310A_PAGE2_SIZE];
    extern byte radio_tx_buf[UHF_LEN];

	extern void vRadioInit(void);
	extern void vRadioClearInterrupt(void);
	extern void vRadioReadAllStatus(void);
	extern void vRadioCmpReg(byte const wr_ptr[], byte rd_ptr[], byte cmp_ptr[], byte length);
	extern void vRadioGoTxInit(void);
	extern void vRadioGoRxInit(void);
    void RF_CMT2310A_Init(void);
    void CMT2310A_Data_Mode(u8 mode);
    void CMT2310A_Set_Freq(u32 freq,DEVSET_ENUM dev);
    void CMT2300A_Read_RxData(void);
    void CMT2300A_Write_TxData(void);
    void CMT2300A_TxDone(void);
    u8 CMT2310A_ReadReg(u8 page,u8 addr);

#endif




