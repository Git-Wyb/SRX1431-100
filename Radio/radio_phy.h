

#ifndef __RADIO_PHY_H

	#define 	__RADIO_PHY_H
	
	#include "radio_hal.h"	
	#include "CMT2310A_def.h"
	#include "CMT2310A_reg.h"
	
	
	extern void 	 	 vRadioSetInt1Sel(unsigned char int1_sel);
	extern void 		 vRadioSetInt2Sel(unsigned char int2_sel);
	extern void 		 vRadioSetInt1Polar(boolean_t int1_polar);
	extern void 		 vRadioSetInt2Polar(boolean_t int2_polar);
	extern void 		 vRadioSetInt3Polar(boolean_t int3_polar);

	
	extern void 		 vRadioRssiUpdateSel(unsigned char sel);
	extern unsigned char bRadioGetRssi(void);
	extern void 		 vRadioRssiConfig(RSSI_CFG rssi_cfg);
	extern void 		 vRadioRssiCalOffset(unsigned char cal_offset);
	
	extern unsigned char bRadioGetLbdValue(void);
	extern void 		 vRadioSetLbdTH(unsigned char lbd_th);	
	extern unsigned char bRadioGetTemperature(void);
	extern boolean_t 	 bRadioApiCommand(unsigned char api_cmd);
	extern void          vRadioCdrModeCfg(enum CDR_MODE cdr_mode);
	extern void 		 vRadioTxRampCfg(boolean_t tx_ramp_en, unsigned short tx_ramp_step);
	extern void 		 vRadioTxGaussianCfg(boolean_t tx_gaus_en, unsigned char tx_gaus_bt);
	extern void 		 vRadioAfcCfg(boolean_t afc_en);
	
	extern unsigned char bRadioGetState(void);
	extern unsigned char bRadioGoSleep(void);
	extern unsigned char bRadioGoStandby(void);
	extern unsigned char bRadioGoTx(void);
	extern unsigned char bRadioGoRx(void);
	extern unsigned char bRadioGoTxFS(void);
	extern unsigned char bRadioGoRxFS(void);
	
	
	extern void 		 vRadioSetFifoTH(unsigned short int fifo_th);
	extern void 		 vRadioFifoRetent(boolean_t cfg_en);
	extern void 		 vRadioFifoAutoClearGoRx(boolean_t cfg_en);
	extern void 		 vRadioFifoAutoRestoreWhenTxDone(boolean_t cfg_en);
	extern void 		 vRadioFifoMerge(boolean_t cfg_en);
	extern void 		 vRadioFifoTRxUsageSel(boolean_t cfg_tx);
	extern void 		 vRadioFifoGetStatus(FIFO_STATUS_FLG *fifo_status);
	extern void 		 vRadioClearTxFifo(void);
	extern void 		 vRadioClearRxFifo(void);
	extern void 		 vRadioManualResetTxFifoPointer(void);
	
	extern void 		 vRadioInterruptSoucreCfg(INT_SRC_CFG *int_src_ctrl);
	extern void 		 vRadioInterruptSoucreFlag(INT_SRC_FLG *int_src_flag);
	extern void 		 vRadioInterruptSoucreClear(INT_SRC_CLR *int_src_clr);


	extern void 		 vRadioConfigPageReg(byte page_sel, unsigned char const reg_ptr[], unsigned char reg_len);
	extern void 		 vRadioReadPageReg(byte page_sel, unsigned char reg_ptr[], unsigned char reg_len);
	extern boolean_t 	 bRadioIsExist(void);
	extern uint32_t 	 lRadioChipVersion(void);

#endif	



