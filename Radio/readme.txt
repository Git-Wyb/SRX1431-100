change list:

2022-01-20
1. modify vRadioSetPayloadLength
2. modify vRadioGetPayloadLength
3. add vRadioAfcCfg, in radio_phy

2022-02-21
1. modify vRadioCsmaEnable, @radio_mac.c

2022-02-25
1. add CMT2310A_ADDR_FIELD_EN at reg_CMT2310A_CTL_REG_63, in CMT2310A_reg.h
2. add ADDR_FIELD_EN, at FRAME_CFG.FRAME_CFG1_u, for node address field, in CMT2310A_def.h

2022-03-18
1. modify vRadioReadTxFifo, @radio_hal.c 

2022-04-24
1. modify WORK_MODE_CFG.WORK_MODE_CFG6_u, add FREQ_SW_STATE(bit5), @CMT2310A_def.h
2. modify vRadioCfgWorkMode, @radio_mac.c, delete set FREQ_DONE_TIMES, because FREQ_DONE_TIMES is only for read.
3. add bRadioGetHopDoneTimes @radio_mac.c 

2022-06-07
1. fixed vRadioSetAntSwitch config error at radio_hal.c

2022-06-17
1. add vRadioXoWaitCfg at radio_hal.c

2022-08-05
1. modify vRadioReadTxFifo & vRadioWriteFifo, @radio_hal.c 

2022-08-26
1. modify vRadioReadFifo, vRadioWriteFifo & vRadioReadTxFifo, for M_FIFO_TX_RX_SEL configuartion, @radio_hal.c
2. modify vRadioFifoTRxUsageSel's decription, TRUE for RX, FALSE for TX