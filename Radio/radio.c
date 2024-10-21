#include "radio.h"
#include "ram.h"

CMT2310A_CFG	g_radio;					//
uint32_t 		g_chip_id = 0x00000000;

unsigned char g_reg_read_buf[128];
byte    radio_rx_buf[UHF_LEN];
byte 	radio_tx_buf[UHF_LEN];

/******************************
**Name:  vRadioInit
**Func:  Radio config spi & reset
**Input: None
*Output: None
********************************/
void RF_CMT2310A_Init(void)
{
    while(1)
	{
        vRadioSoftReset();
        vRadioPowerUpBoot();
        delay1ms(10);
        g_chip_id = lRadioChipVersion();
        if(0x00231000==(g_chip_id&0x00FFFF00))
            break;
    }

    vRadioInit();
    //g_radio.frame_cfg.PAYLOAD_LENGTH = UHF_LEN;
    //vRadioSetPayloadLength(&g_radio.frame_cfg);
    //vRadioSetInt1Sel(CMT2310A_INT_PKT_DONE);
    //vRadioSetInt2Sel(CMT2310A_INT_RX_FIFO_WBYTE);
    //bRadioGoRx();
}

void vRadioInit(void)
{
	byte fw_rev;
	vRadioSoftReset();

	vRadioConfigPageReg(0, g_cmt2310a_page0, CMT2310A_PAGE0_SIZE);		//config page 0
	vRadioConfigPageReg(1, g_cmt2310a_page1, CMT2310A_PAGE1_SIZE);   	//config page 1

	vRadioSetNirq(CMT2310A_nIRQ_SEL);	//for TCXO need cofig as nIRQ pin at first
	//vRadioTcxoDrvSel(0);				//drive power

	fw_rev = (byte)g_chip_id;			//dealwith Xtal
	switch(fw_rev)
		{
		case 0xC0: vRadioXoWaitCfg(RADIO_CGU_DIV4); break;
		default: break;
		}


	vRadioPowerUpBoot();
	delay1ms(10);

	bRadioGoStandby();
	delay1ms(2);
	bRadioApiCommand(0x02);				//
	delay1ms(10);
	bRadioApiCommand(0x01);				//IR Calibration, need some times

	vRadioCapLoad(2);					//Xo Cap

	//GPIOn default setting
	vRadioSetGpio0(CMT2310A_GPIO0_SEL);
	vRadioSetGpio1(CMT2310A_GPIO1_SEL);
	vRadioSetGpio2(CMT2310A_GPIO2_SEL);
	vRadioSetGpio3(CMT2310A_GPIO3_INT1);
	vRadioSetGpio4(CMT2310A_GPIO4_DCLK);
	vRadioSetGpio5(CMT2310A_GPIO5_DOUT);


	//INT1 = RX_FIFO_WBYTE,   INT2 = PKT_DONE
	vRadioSetInt1Sel(INT_SRC_RX_FIFO_WBYTE);
	vRadioSetInt2Sel(INT_SRC_PKT_DONE);
	vRadioSetInt1Polar(FALSE);
	vRadioSetInt2Polar(FALSE);
	vRadioSetInt3Polar(FALSE);

	//interrupt source enable config
	g_radio.int_src_en._BITS.PKT_DONE_EN   		= 1;
	g_radio.int_src_en._BITS.CRC_PASS_EN   		= 1;
	g_radio.int_src_en._BITS.ADDR_PASS_EN  		= 0;
	g_radio.int_src_en._BITS.SYNC_PASS_EN  		= 1;
	g_radio.int_src_en._BITS.PREAM_PASS_EN 		= 1;
	g_radio.int_src_en._BITS.TX_DONE_EN    		= 1;
	g_radio.int_src_en._BITS.RX_TOUT_EN    		= 1;
	g_radio.int_src_en._BITS.LD_STOP_EN    		= 0;
	g_radio.int_src_en._BITS.LBD_STOP_EN   		= 0;
	g_radio.int_src_en._BITS.LBD_STAT_EN   		= 0;
	g_radio.int_src_en._BITS.PKT_ERR_EN    		= 0;
	g_radio.int_src_en._BITS.RSSI_COLL_EN  		= 0;
	g_radio.int_src_en._BITS.OP_CMD_FAILED_EN 	= 0;
	g_radio.int_src_en._BITS.RSSI_PJD_EN   		= 0;
	g_radio.int_src_en._BITS.SEQ_MATCH_EN  		= 0;
	g_radio.int_src_en._BITS.NACK_RECV_EN       = 0;
	g_radio.int_src_en._BITS.TX_RESEND_DONE_EN  = 0;
	g_radio.int_src_en._BITS.ACK_RECV_FAILED_EN = 0;
	g_radio.int_src_en._BITS.TX_DC_DONE_EN      = 0;
	g_radio.int_src_en._BITS.CSMA_DONE_EN       = 0;
	g_radio.int_src_en._BITS.CCA_STAT_EN        = 0;
	g_radio.int_src_en._BITS.API_DONE_EN        = 0;
	g_radio.int_src_en._BITS.TX_FIFO_TH_EN		= 1;
	g_radio.int_src_en._BITS.TX_FIFO_NMTY_EN	= 1;
	g_radio.int_src_en._BITS.TX_FIFO_FULL_EN	= 1;
	g_radio.int_src_en._BITS.RX_FIFO_OVF_EN		= 1;
	g_radio.int_src_en._BITS.RX_FIFO_TH_EN		= 1;
	g_radio.int_src_en._BITS.RX_FIFO_NMTY_EN	= 1;
	g_radio.int_src_en._BITS.RX_FIFO_FULL_EN 	= 1;
	vRadioInterruptSoucreCfg(&g_radio.int_src_en);

	//packet preamble config
	g_radio.preamble_cfg.PREAM_LENG_UNIT = 0;					//8-bits mode
	g_radio.preamble_cfg.PREAM_VALUE     = 0x00;				//
	g_radio.preamble_cfg.RX_PREAM_SIZE   = 0;					//
	g_radio.preamble_cfg.TX_PREAM_SIZE   = 0;
	vRadioCfgPreamble(&g_radio.preamble_cfg);

	//packet syncword config
	g_radio.sync_cfg.SYN_CFG_u._BITS.SYNC_MAN_EN   = 0;			//disable syncword manchester coding
	g_radio.sync_cfg.SYN_CFG_u._BITS.SYNC_SIZE     = 2;			//(N+1).enable 3 bytes for syncword
	g_radio.sync_cfg.SYN_CFG_u._BITS.SYNC_TOL      = 0;
	g_radio.sync_cfg.SYN_CFG_u._BITS.SYNC_MODE_SEL = 0;			//normal packet
	g_radio.sync_cfg.SYNC_VALUE[0] = 0x00;
	g_radio.sync_cfg.SYNC_VALUE[1] = 0x00;
	g_radio.sync_cfg.SYNC_VALUE[2] = 0x00;
	g_radio.sync_cfg.SYNC_VALUE_SEL= 0;							//select SYN_VAL
	vRadioCfgSyncWord(&g_radio.sync_cfg);

	//packet node address config
	g_radio.addr_cfg.ADDR_CFG_u._BITS.ADDR_DET_MODE = 0;		//disable Node Address
	vRadioCfgNodeAddr(&g_radio.addr_cfg);

	//packet crc config
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_EN         = 0;			//enable crc
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_BIT_ORDER  = 0;
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_REFIN      = 0;
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_RANGE      = 0;
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_BIT_INV    = 0;
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_BYTE_SWAP  = 0;
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_REFOUT     = 0;			//whole payload
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRCERR_CLR_FIFO_EN  = 0;	//note: need ative FIFO_AUTO_CLR_RX_EN = 1 or call vRadioFifoAutoClearGoRx(1)
	g_radio.crc_cfg.CRC_CFG_u._BITS.CRC_SIZE       = 1;			//crc-16 mode
	g_radio.crc_cfg.CRC_POLY_u.u32_POLY = 0x10210000;
	g_radio.crc_cfg.CRC_SEED_u.u32_SEED = 0x00000000;
	vRadioCfgCrc(&g_radio.crc_cfg);

	//packet coding format
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.MANCH_EN          = 0;
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.MANCH_TYPE        = 0;
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.WHITEN_EN         = 0;
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.WHITEN_TYPE       = 0;
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.WHITEN_SEED_TYP   = 0;
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.FEC_EN            = 0;
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.FEC_RSC_NRNSC_SEL = 0;
	g_radio.coding_format_cfg.CODING_FORMAT_CFG_u._BITS.FEC_TICC          = 0;
	g_radio.coding_format_cfg.WHITEN_SEED  = 0x01FF;
	g_radio.coding_format_cfg.FEC_PAD_CODE = 0;
	vRadioCfgCodeFormat(&g_radio.coding_format_cfg);

	//packet frame format
	g_radio.frame_cfg.DATA_MODE = 2;								//0=direct mode, 	2=packet mode
	g_radio.frame_cfg.FRAME_CFG1_u._BITS.PKT_TYPE 	       = 0;		//fixd-length packet mode
	g_radio.frame_cfg.FRAME_CFG1_u._BITS.PAYLOAD_BIT_ORDER = 0;		//msb first
	g_radio.frame_cfg.FRAME_CFG1_u._BITS.ADDR_LEN_CONF     = 0;
	g_radio.frame_cfg.FRAME_CFG1_u._BITS.PAGGYBACKING_EN   = 0;
	g_radio.frame_cfg.FRAME_CFG1_u._BITS.LENGTH_SIZE 	   = 0;
	g_radio.frame_cfg.FRAME_CFG1_u._BITS.INTERLEAVE_EN     = 0;		//note: when FEC enable, INTERLEAVE_EN should be set 1

	g_radio.frame_cfg.FRAME_CFG2_u._BITS.TX_PREFIX_TYPE    = TX_PREFIX_SEL_PREAMBLE;		//transmit preamble
	g_radio.frame_cfg.FRAME_CFG2_u._BITS.SEQNUM_EN  	   = 0;
	g_radio.frame_cfg.FRAME_CFG2_u._BITS.SEQNUM_AUTO_INC   = 0;
	g_radio.frame_cfg.FRAME_CFG2_u._BITS.SEQNUM_SIZE	   = 0;
	g_radio.frame_cfg.FRAME_CFG2_u._BITS.SEQNUM_MACH_EN    = 0;
	g_radio.frame_cfg.FRAME_CFG2_u._BITS.FCS2_EN    	   = 0;

	g_radio.frame_cfg.TX_PKT_NUM     = 0;
	g_radio.frame_cfg.TX_PKT_GAP     = 0;
	g_radio.frame_cfg.FCS2_TX_IN     = 0;
	g_radio.frame_cfg.PAYLOAD_LENGTH = UHF_LEN;
	vRadioCfgFrameFormat(&g_radio.frame_cfg);

	//Run Mode Config
	g_radio.word_mode_cfg.WORK_MODE_CFG1_u._BITS.TX_DC_EN          = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG1_u._BITS.TX_ACK_EN         = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG1_u._BITS.TX_DC_PERSIST_EN  = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG1_u._BITS.TX_AUTO_HOP_EN    = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG1_u._BITS.TX_EXIT_STATE     = EXIT_TO_READY;

	g_radio.word_mode_cfg.WORK_MODE_CFG2_u._BITS.RX_DC_EN          = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG2_u._BITS.RX_AUTO_HOP_EN    = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG2_u._BITS.RX_ACK_EN         = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG2_u._BITS.RX_TIMER_EN       = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG2_u._BITS.RX_EXIT_STATE     = EXIT_TO_READY;
	g_radio.word_mode_cfg.WORK_MODE_CFG2_u._BITS.CSMA_EN           = 0;

	g_radio.word_mode_cfg.WORK_MODE_CFG3_u._BITS.PKT_DONE_EXIT_EN  = 0;			//depend on RX_EXIT_STATE
	g_radio.word_mode_cfg.WORK_MODE_CFG3_u._BITS.RX_HOP_SLP_MODE   = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG3_u._BITS.SLP_MODE          = 0;

	g_radio.word_mode_cfg.WORK_MODE_CFG4_u._BITS.LFCLK_OUT_EN      = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG4_u._BITS.LFCLK_SEL         = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG4_u._BITS.SLEEP_TIMER_EN    = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG4_u._BITS.TIMER_RAND_MODE   = 0;

	g_radio.word_mode_cfg.WORK_MODE_CFG5_u._BITS.CSMA_CCA_MODE     = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG5_u._BITS.CSMA_CCA_WIN_SEL  = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG5_u._BITS.CSMA_CCA_INT_SEL  = 0;
	g_radio.word_mode_cfg.WORK_MODE_CFG5_u._BITS.CSMA_PERSIST_EN   = 0;

	g_radio.word_mode_cfg.FREQ_CHANL_NANU  = 0;
	g_radio.word_mode_cfg.FREQ_DONE_TIMES  = 0;
	g_radio.word_mode_cfg.FREQ_SPACE       = 0;
	g_radio.word_mode_cfg.FREQ_TIMES       = 0;
	g_radio.word_mode_cfg.SLEEP_TIMER_M    = 0;
	g_radio.word_mode_cfg.SLEEP_TIMER_R    = 0;
	g_radio.word_mode_cfg.RX_TIMER_T1_M    = 0;					//M*2^(R+1)*5us=M*2^R*10us,
	g_radio.word_mode_cfg.RX_TIMER_T1_R    = 0;					//R=7, unit=0.64ms
	g_radio.word_mode_cfg.RX_TIMER_T2_M    = 0;
	g_radio.word_mode_cfg.RX_TIMER_T2_R    = 0;
	g_radio.word_mode_cfg.RX_TIMER_CSMA_M  = 0;
	g_radio.word_mode_cfg.RX_TIMER_CSMA_R  = 0;
	g_radio.word_mode_cfg.TX_DC_TIMES      = 0;
	g_radio.word_mode_cfg.TX_RS_TIMES      = 0;
	g_radio.word_mode_cfg.CSMA_TIMES       = 0;
	g_radio.word_mode_cfg.SLEEP_TIMER_CSMA_M = 0;
	g_radio.word_mode_cfg.SLEEP_TIMER_CSMA_R = 0;
	vRadioCfgWorkMode(&g_radio.word_mode_cfg);

	//FIFO Init
	vRadioFifoMerge(FALSE);
	vRadioSetFifoTH(30);
	vRadioClearRxFifo();										//reset & clear fifo
	vRadioClearTxFifo();

	vRadioFifoAutoClearGoRx(TRUE);								//when crc error, need to auto clear fifo, should enable

	vRadioRssiUpdateSel(CMT2310A_RSSI_UPDATE_ALWAYS);

	vRadioSetAntSwitch(FALSE, FALSE);							//

	vRadioDcdcCfg(TRUE);										//dc-dc off
}

void vRadioClearInterrupt(void)
{
	vRadioInterruptSoucreFlag(&g_radio.int_src_flag);

	g_radio.int_src_clear._BITS.SLEEP_TMO_CLR = g_radio.int_src_flag._BITS.SLEEP_TMO_FLG;
	g_radio.int_src_clear._BITS.RX_TMO_CLR    = g_radio.int_src_flag._BITS.RX_TMO_FLG;
	g_radio.int_src_clear._BITS.TX_DONE_CLR   = g_radio.int_src_flag._BITS.TX_DONE_FLG;

	g_radio.int_src_clear._BITS.PKT_DONE_CLR  = g_radio.int_src_flag._BITS.PKT_DONE_FLG;
	g_radio.int_src_clear._BITS.CRC_PASS_CLR  = g_radio.int_src_flag._BITS.CRC_PASS_FLG;
	g_radio.int_src_clear._BITS.ADDR_PASS_CLR = g_radio.int_src_flag._BITS.ADDR_PASS_FLG;
	g_radio.int_src_clear._BITS.SYNC_PASS_CLR = g_radio.int_src_flag._BITS.SYNC_PASS_FLG|g_radio.int_src_flag._BITS.SYNC1_PASS_FLG;
	g_radio.int_src_clear._BITS.PREAM_PASS_CLR= g_radio.int_src_flag._BITS.PREAM_PASS_FLG;

	g_radio.int_src_clear._BITS.LBD_STAT_CLR      = g_radio.int_src_flag._BITS.LBD_STATUS_FLG;
	g_radio.int_src_clear._BITS.PKT_ERR_CLR       = g_radio.int_src_flag._BITS.PKT_ERR_FLG;
	g_radio.int_src_clear._BITS.RSSI_COLL_CLR     = g_radio.int_src_flag._BITS.RSSI_COLL_FLG;
	g_radio.int_src_clear._BITS.OP_CMD_FAILED_CLR = g_radio.int_src_flag._BITS.OP_CMD_FAILED_FLG;
	g_radio.int_src_clear._BITS.ANT_LOCK_CLR      = g_radio.int_src_flag._BITS.ANT_LOCK_FLG;

	g_radio.int_src_clear._BITS.SEQ_MATCH_CLR       = g_radio.int_src_flag._BITS.SEQ_MATCH_FLG;
	g_radio.int_src_clear._BITS.NACK_RECV_CLR       = g_radio.int_src_flag._BITS.NACK_RECV_FLG;
	g_radio.int_src_clear._BITS.TX_RESEND_DONE_CLR  = g_radio.int_src_flag._BITS.TX_RESEND_DONE_FLG ;
	g_radio.int_src_clear._BITS.ACK_RECV_FAILED_CLR = g_radio.int_src_flag._BITS.ACK_RECV_FAILED_FLG;
	g_radio.int_src_clear._BITS.TX_DC_DONE_CLR      = g_radio.int_src_flag._BITS.TX_DC_DONE_FLG;
	g_radio.int_src_clear._BITS.CSMA_DONE_CLR       = g_radio.int_src_flag._BITS.CSMA_DONE_FLG;
	g_radio.int_src_clear._BITS.CCA_STATUS_CLR      = g_radio.int_src_flag._BITS.CCA_STATUS_FLG;
	g_radio.int_src_clear._BITS.API_DONE_CLR        = g_radio.int_src_flag._BITS.API_DONE_FLG;

 	vRadioInterruptSoucreClear(&g_radio.int_src_clear);
}

void vRadioReadAllStatus(void)
{
	bRadioGetState();									//read work status
	vRadioFifoGetStatus(&g_radio.fifo_status_flag);		//read fifo status
	vRadioInterruptSoucreFlag(&g_radio.int_src_flag);	//read interrupt flag

	bRadioReadReg(CMT2310A_CTL_REG_04);					//get GPIO1/GPIO0 selection
	bRadioReadReg(CMT2310A_CTL_REG_05);					//get GPIO3/GPIO2 selection
	bRadioReadReg(CMT2310A_CTL_REG_06);					//get GPIO5/GPIO4 selection

	bRadioReadReg(CMT2310A_CTL_REG_16);					//get INT1 selection
	bRadioReadReg(CMT2310A_CTL_REG_17);					//get INT2 selection
}

void vRadioCmpReg(byte const wr_ptr[], byte rd_ptr[], byte cmp_ptr[], byte length)
{
	byte i;

	for(i=0; i<length; i++)
		{
		if(wr_ptr[i]!=rd_ptr[i])
			cmp_ptr[i] = 0xFF;
		else
			cmp_ptr[i] = 0x00;
		}
}

void vRadioGoTxInit(void)
{


}

void vRadioGoRxInit(void)
{


}

void CMT2300A_Read_RxData(void)
{
    vRadioReadFifo(radio_rx_buf, UHF_LEN);
    vRadioClearRxFifo();
    vRadioClearInterrupt();
    bRadioGoRx();
}

void CMT2300A_Write_TxData(void)
{
    vRadioSetInt1Sel(CMT2310A_INT_TX_DONE);
    vRadioSetInt2Sel(CMT2310A_INT_TX_FIFO_NMTY);
    vRadioWriteFifo(radio_tx_buf, UHF_LEN);
    bRadioGoTx();
}
void CMT2300A_TxDone(void)
{
    bRadioGoStandby();
    vRadioClearTxFifo();
    vRadioClearInterrupt();
    flag_tx_done = 1;
}

void CMT2310A_Set_Freq(u32 freq,DEVSET_ENUM dev) //dev -> The value depends on RFPDK how much the export is set to
{
    switch(freq)
    {
        case 426075000:
            g_cmt2310a_page1[16] = 0x6A;
            g_cmt2310a_page1[17] = 0xCC;
            g_cmt2310a_page1[18] = 0x4C;
            g_cmt2310a_page1[19] = 0x08;
            g_cmt2310a_page1[49] = 0x6A;
            g_cmt2310a_page1[50] = 0xCC;
            g_cmt2310a_page1[51] = 0xCC;
            g_cmt2310a_page1[52] = 0xD8;
            g_cmt2310a_page1[73] = 0xB3; //426MHz, Dev 2.4K and Dev 2.0K alike.
        break;

        case 426100000:
            g_cmt2310a_page1[16] = 0x6A;
            g_cmt2310a_page1[17] = 0x66;
            g_cmt2310a_page1[18] = 0x66;
            g_cmt2310a_page1[19] = 0x08;
            g_cmt2310a_page1[49] = 0x6A;
            g_cmt2310a_page1[50] = 0x66;
            g_cmt2310a_page1[51] = 0xE6;
            g_cmt2310a_page1[52] = 0xD8;
            g_cmt2310a_page1[73] = 0xB3;
        break;

        case 426200000:
            g_cmt2310a_page1[16] = 0x6A;
            g_cmt2310a_page1[17] = 0xCC;
            g_cmt2310a_page1[18] = 0xCC;
            g_cmt2310a_page1[19] = 0x08;
            g_cmt2310a_page1[49] = 0x6A;
            g_cmt2310a_page1[50] = 0xCC;
            g_cmt2310a_page1[51] = 0x4C;
            g_cmt2310a_page1[52] = 0xD9;
            g_cmt2310a_page1[73] = 0xB3;
        break;

        case 429175000:
            g_cmt2310a_page1[16] = 0x6B;
            g_cmt2310a_page1[17] = 0x33;
            g_cmt2310a_page1[18] = 0xB3;
            g_cmt2310a_page1[19] = 0x04;
            g_cmt2310a_page1[49] = 0x6B;
            g_cmt2310a_page1[50] = 0x33;
            g_cmt2310a_page1[51] = 0x33;
            g_cmt2310a_page1[52] = 0xD5;
        break;

        case 429200000:
            g_cmt2310a_page1[16] = 0x6B;
            g_cmt2310a_page1[17] = 0xCC;
            g_cmt2310a_page1[18] = 0xCC;
            g_cmt2310a_page1[19] = 0x04;
            g_cmt2310a_page1[49] = 0x6B;
            g_cmt2310a_page1[50] = 0xCC;
            g_cmt2310a_page1[51] = 0x4C;
            g_cmt2310a_page1[52] = 0xD5;
        break;

        case 429350000:
            g_cmt2310a_page1[16] = 0x6B;
            g_cmt2310a_page1[17] = 0x66;
            g_cmt2310a_page1[18] = 0x66;
            g_cmt2310a_page1[19] = 0x05;
            g_cmt2310a_page1[49] = 0x6B;
            g_cmt2310a_page1[50] = 0x66;
            g_cmt2310a_page1[51] = 0xE6;
            g_cmt2310a_page1[52] = 0xD5;
        break;

        case 429550000:
            g_cmt2310a_page1[16] = 0x6B;
            g_cmt2310a_page1[17] = 0x33;
            g_cmt2310a_page1[18] = 0x33;
            g_cmt2310a_page1[19] = 0x06;
            g_cmt2310a_page1[49] = 0x6B;
            g_cmt2310a_page1[50] = 0x33;
            g_cmt2310a_page1[51] = 0xB3;
            g_cmt2310a_page1[52] = 0xD6;
        break;

        default:
            break;
    }
    if(freq > 429000000)
    {
        if(dev == DEV_2_0K)
            g_cmt2310a_page1[73] = 0xB3; //429MHz, Dev 2.4K -> 0x33,Dev 2.0K -> 0xB3.
        else if(dev == DEV_2_4K)
            g_cmt2310a_page1[73] = 0x33;
    }
    vRadioRegPageSel(1);
    bRadioSetReg(16,g_cmt2310a_page1[16],0xFF);
    bRadioSetReg(17,g_cmt2310a_page1[17],0xFF);
    bRadioSetReg(18,g_cmt2310a_page1[18],0xFF);
    bRadioSetReg(19,g_cmt2310a_page1[19],0xFF);
    bRadioSetReg(49,g_cmt2310a_page1[49],0xFF);
    bRadioSetReg(50,g_cmt2310a_page1[50],0xFF);
    bRadioSetReg(51,g_cmt2310a_page1[51],0xFF);
    bRadioSetReg(52,g_cmt2310a_page1[52],0xFF);
    bRadioSetReg(73,g_cmt2310a_page1[73],0xFF);
    vRadioRegPageSel(0);
}

void CMT2310A_Data_Mode(u8 mode)
{
    switch(mode)
    {
        case 0: //Direct 直通模式
            g_cmt2310a_page0[0] = 0x10;
            g_cmt2310a_page1[23] = 0x60;
        break;

        case 1: //Packet 包模式
            g_cmt2310a_page0[0] = 0x12;
            g_cmt2310a_page1[23] = 0xE0;
        break;
    }
}
