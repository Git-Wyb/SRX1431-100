

#ifndef __RADIO_HAL_H

	#define 	__RADIO_HAL_H

	#include "CMT2310A_def.h"
	#include "CMT2310A_reg.h"

	#define		RADIO_CGU_DIV1		0
	#define		RADIO_CGU_DIV4		1
	#define		RADIO_CGU_DIV8		2

	extern void 		 vRadioGpioInit(void);

	extern unsigned char bRadioReadReg(unsigned char addr);
	extern unsigned char bRadioWriteReg(unsigned char addr, unsigned char reg_dat);
	extern unsigned char bRadioSetReg(unsigned char addr, unsigned char set_bits, unsigned char mask_bits);

	extern void 		 vRadioLoadRegs(unsigned char sta_adr, unsigned char *ptr_buf, unsigned char length);
	extern void 		 vRadioStoreRegs(unsigned char sta_adr, unsigned char *ptr_buf, unsigned char length);

	extern void 		 vRadioBurstReadRegs(unsigned char *ptr_buf, unsigned char length);
	extern void 		 vRadioBurstWriteRegs(unsigned char *ptr_buf, unsigned char length);

	extern void 		 vRadioReadFifo(unsigned char *ptr_fifo, unsigned char length);
	extern void 		 vRadioWriteFifo(unsigned char *ptr_fifo, unsigned char length);
	extern void 		 vRadioReadTxFifo(unsigned char *ptr_fifo, unsigned char length);


	extern void 		 vRadioSpiModeSel(boolean_t spi_mod);
	extern void 		 vRadioSetTxDin(boolean_t cfg_din, unsigned char pin_sel);
	extern void 		 vRadioSetDigClkOut(boolean_t cfg_out);
	extern void 		 vRadioSetLfxoPad(boolean_t cfg_lfxo);
	extern void 		 vRadioSetGpio0(unsigned char gpio0_sel);
	extern void 		 vRadioSetGpio1(unsigned char gpio1_sel);
	extern void 		 vRadioSetGpio2(unsigned char gpio2_sel);
	extern void 		 vRadioSetGpio3(unsigned char gpio3_sel);
	extern void 		 vRadioSetGpio4(unsigned char gpio4_sel);
	extern void 		 vRadioSetGpio5(unsigned char gpio5_sel);
	extern void 		 vRadioSetNirq(unsigned char nirq_sel);
	extern void 		 vRadioTcxoDrvSel(unsigned char drv_sel);

	extern void 		 vRadioRegPageSel(unsigned char page_sel);
	extern void 		 vRadioPowerUpBoot(void);
	extern void 		 vRadioSoftReset(void);
	extern void 		 vRadioSetPaOutputMode(boolean_t cfg_en);
	extern void 		 vRadioSetTxDataInverse(boolean_t cfg_en);
  	extern void 		 vRadioSetAntSwitch(boolean_t cfg_en, boolean_t cfg_polar);

	extern void 		 vRadioDcdcCfg(boolean_t on_off);
	extern void 		 vRadioCapLoad(unsigned char cap_value);
	extern void 		 vRadioLfoscCfg(boolean_t on_off);
	extern void 		 vRadioXoWaitCfg(unsigned char div_sel);
#endif



