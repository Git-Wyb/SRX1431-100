

#ifndef __RADIO_MAC_H

	#define 	__RADIO_MAC_H
	
	#include "radio_hal.h"	
	#include "CMT2310A_def.h"
	#include "CMT2310A_reg.h"
	
	
	extern unsigned char  bRadioGetCurrentChannl(void);

	extern void 		  vRadioSetTxSeqNumberInitValue(FRAME_CFG *frm_cfg);
	extern unsigned short wRadioGetTxSeqNumberCurrent(FRAME_CFG *frm_cfg);
 	extern void 		  vRadioSetTxFCS2(FRAME_CFG *frm_cfg);
	extern unsigned char  bRadioGetRxFCS2(FRAME_CFG *frm_cfg);
	extern void 		  vRadioSetPayloadLength(FRAME_CFG *frm_cfg);
	extern unsigned short vRadioGetPayloadLength(FRAME_CFG *frm_cfg);


	extern void 		  vRadioCfgPreamble(PREAMBLE_CFG *prm_ptr);
	extern void 		  vRadioCfgSyncWord(SYNC_CFG *sync_ptr);
	extern void 		  vRadioCfgNodeAddr(ADDR_CFG *node_addr_ptr);
	extern void 		  vRadioCfgCrc(CRC_CFG *crc_ptr);
	extern void			  vRadioCfgCodeFormat(CODING_FORMAT_CFG *code_format_ptr);
	extern void 		  vRadioCfgFrameFormat(FRAME_CFG *frame_format_ptr);
	extern void  		  vRadioCfgWiSunFormat(WI_SUN_CFG *wi_sun_ptr);
	extern void 		  vRadioCdrTracingModeCfg(CDR_TRACING_CFG *cdr_ptr);
		
	extern void 		  vRadioCfgWorkMode(WORK_MODE_CFG *run_mode_ptr);
	extern void 		  vRadioReadRunModeCfg(void);
	extern unsigned char  bRadioGetTxDutyCycleDoneTimes(WORK_MODE_CFG *run_mode_ptr);
	extern unsigned char  bRadioGetTxResendDoneTimes(WORK_MODE_CFG *run_mode_ptr);
	extern unsigned char  bRadioGetCMSADoneTimes(WORK_MODE_CFG *run_mode_ptr);
	
	
	extern void 		  vRadioSendWithAck(boolean_t w_ack, FRAME_CFG *frame_format_ptr);
	extern void 		  vRadioEnableTxAck(boolean_t en_flg, WORK_MODE_CFG *run_mode_ptr);
	extern void 		  vRadioEnableRxAck(boolean_t en_flg, WORK_MODE_CFG *run_mode_ptr);
	extern unsigned char  bRadioGetFreqChanl(void);
	extern unsigned char  bRadioGetHopDoneTimes(void);
	extern void 		  vRadioCsmaEnable(boolean_t on_off, WORK_MODE_CFG *run_mode_ptr);
	extern void 		  vRadioSetRssiAbsThValue(signed char rssi);
	extern void 		  vRadioSetPjdDetWin(unsigned char pjd_win);
	


#endif	


