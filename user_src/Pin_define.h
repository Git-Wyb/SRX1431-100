/***********************************************************************/
/*  File Name   :Pin_Define.h                                          */
/*  DATE        :Mar, 2014                                             */
/*  Programmer	:xiang 'R                                              */
/*  CPU TYPE    :STM8L151G6     Crystal: 16M HSI                       */
/*  DESCRIPTION :                                                      */
/*  Mark        :ver 1.0                                               */
/***********************************************************************/

//������IO����

/******************������data�Ĵ���************************************/
/* ADF7012 register interface */
#define ADF7021_SCLK            PB_ODR_ODR4     // Output   ʱ����
#define ADF7021_SDATA           PB_ODR_ODR6     // Output   ������
#define ADF7021_SLE             PB_ODR_ODR7     // Output   Ƭѡ
#define ADF7021_SREAD           PB_IDR_IDR5     // Input
#define ADF7021_CE              PD_ODR_ODR4     // Output   ADF7012ʹ�ܽ�
#define ADF7021_MUXOUT          PB_IDR_IDR1    // Input

/* Other ADF7021 connections */
#define ADF7021_DATA_tx         PB_ODR_ODR3     // Output   ����DATA��
#define ADF7021_DATA_rx         PB_IDR_IDR3     // Input   ����DATA��
#define ADF7021_DATA_CLK        PB_IDR_IDR2     // ����   ����ͬ��ʱ����


     /* ���Ż�ʹ�õ�IO*/
#define HA_L_signal             PA_IDR_IDR4      // Input   HA �����ź�   �͵�ƽ��Ч
#define HA_ERR_signal           PA_IDR_IDR5    // Input   HA �쳣�ź�  �͵�ƽ��Ч
#define HA_Sensor_signal        PA_IDR_IDR3    // Input   HA �������ź�  �͵�ƽ��Ч
#define Receiver_Login          PC_IDR_IDR6    // Input   ���Ż���¼��   �͵�ƽ��Ч
//#define Receiver_Buzzer         PC_ODR_ODR5  // Output   ���Ż�������  �ߵ�ƽ��Ч
#define Receiver_vent           PC_IDR_IDR5    // Input   ���Ż���������ON/OFF
#define PIN_BEEP                PA_ODR_ODR0     // Output   ������
#define Receiver_LED_OUT        PB_ODR_ODR1  // Output   ���Ż��̵����������  �ߵ�ƽ��Ч
#define Receiver_LED_TX         PB_ODR_ODR2 // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_LED_RX         PB_ODR_ODR3  // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_OUT_OPEN       PD_ODR_ODR3  // Output   ���Ż��̵���OPEN  �ߵ�ƽ��Ч
#define Receiver_OUT_CLOSE      PD_ODR_ODR2  // Output   ���Ż��̵���close  �ߵ�ƽ��Ч
#define Receiver_OUT_STOP       PD_ODR_ODR1  // Output   ���Ż��̵���stop  �ߵ�ƽ��Ч
#define Receiver_OUT_VENT       PD_ODR_ODR0   // Output ���Ż��̵���VENT  �ߵ�ƽ��Ч
#define Inverters_OUT           PA_IDR_IDR2     // ����   �̵�������źŷ���   �͵�ƽ��Ч

#define  Receiver_test          PB_IDR_IDR0    // Input ���Ż����Խ�  �ߵ�ƽ��Ч

/************************�����Ƿ���Ĵ���*****************************/
/* ADF7012 register interface */
#define ADF7021_SCLK_direc      PB_DDR_DDR4     // Output   ʱ����
#define ADF7021_SDATA_direc     PB_DDR_DDR6     // Output   ������
#define ADF7021_SLE_direc       PB_DDR_DDR7     // Output   Ƭѡ
#define ADF7021_SREAD_direc     PB_DDR_DDR5     // Input
#define ADF7021_CE_direc        PD_DDR_DDR4     // Output   ADF7012ʹ�ܽ�
#define ADF7021_MUXOUT_direc    PB_DDR_DDR1    // Input

/* Other ADF7021 connections */
#define ADF7021_DATA_direc      PB_DDR_DDR3     // Output/Input   ����DATA��
#define ADF7021_DATA_IO         PB_DDR_DDR3     // Output/Input   ����DATA��
#define ADF7021_DATA_CLK_direc  PB_DDR_DDR2     // ����   ����ͬ��ʱ����


     /* ���Ż�ʹ�õ�IO*/
#define HA_L_signal_direc       PA_DDR_DDR4      // Input   HA �����ź�   �͵�ƽ��Ч
#define HA_ERR_signal_direc     PA_DDR_DDR5    // Input   HA �쳣�ź�  �͵�ƽ��Ч
#define HA_Sensor_signal_direc  PA_DDR_DDR3    // Input   HA �������ź�  �͵�ƽ��Ч
#define Receiver_Login_direc    PA_DDR_DDR2    // Input   ���Ż���¼��   �͵�ƽ��Ч
//#define Receiver_Buzzer_direc   PC_DDR_DDR5  // Output   ���Ż�������  �ߵ�ƽ��Ч
#define Receiver_vent_direc     PD_DDR_DDR0  // Input   ���Ż���������ON/OFF
#define PIN_BEEP_direc          PA_DDR_DDR0     // Output   ������
#define Receiver_LED_OUT_direc  PB_DDR_DDR1  // Output   ���Ż��̵����������  �ߵ�ƽ��Ч
#define Receiver_LED_TX_direc   PB_DDR_DDR2 // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_LED_RX_direc   PB_DDR_DDR3  // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_OUT_OPEN_direc PD_DDR_DDR3  // Output   ���Ż��̵���OPEN  �ߵ�ƽ��Ч
#define Receiver_OUT_CLOSE_direc PD_DDR_DDR2  // Output   ���Ż��̵���close  �ߵ�ƽ��Ч
#define Receiver_OUT_STOP_direc PD_DDR_DDR1  // Output   ���Ż��̵���stop  �ߵ�ƽ��Ч
#define Receiver_OUT_VENT_direc PD_DDR_DDR0   // Output ���Ż��̵���VENT  �ߵ�ƽ��Ч
#define Inverters_OUT_direc     PA_DDR_DDR3     // ����   �̵�������źŷ���   �͵�ƽ��Ч

#define Receiver_test_direc     PB_DDR_DDR0    // Input ���Ż����Խ�  �ߵ�ƽ��Ч

/************************�����ǿ��ƼĴ���*****************************/
/* ADF7012 register interface */
#define ADF7021_SCLK_CR1        PB_CR1_C14     // Output   ʱ����
#define ADF7021_SDATA_CR1       PB_CR1_C16     // Output   ������
#define ADF7021_SLE_CR1         PB_CR1_C17     // Output   Ƭѡ
#define ADF7021_SREAD_CR1       PB_CR1_C15     // Input
#define ADF7021_CE_CR1          PD_CR1_C14     // Output   ADF7012ʹ�ܽ�
#define ADF7021_MUXOUT_CR1      PB_CR1_C11    // Input

/* Other ADF7021 connections */
#define ADF7021_DATA_CR1        PB_CR1_C13     // Output/Input   ����DATA��
#define ADF7021_DATA_CLK_CR1    PB_CR1_C12     // ����   ����ͬ��ʱ����


     /* ���Ż�ʹ�õ�IO*/
#define HA_L_signal_CR1         PA_CR1_C14      // Input   HA �����ź�   �͵�ƽ��Ч
#define HA_ERR_signal_CR1       PA_CR1_C15    // Input   HA �쳣�ź�  �͵�ƽ��Ч
#define HA_Sensor_signal_CR1    PA_CR1_C13    // Input   HA �������ź�  �͵�ƽ��Ч
#define Receiver_Login_CR1      PA_CR1_C12    // Input   ���Ż���¼��   �͵�ƽ��Ч
//#define Receiver_Buzzer_CR1     PC_CR1_C15  // Output   ���Ż�������  �ߵ�ƽ��Ч
#define Receiver_vent_CR1       PC_CR1_C15  // Input   ���Ż���������ON/OFF
#define PIN_BEEP_CR1            PA_CR1_C10     // Output   ������
#define Receiver_LED_OUT_CR1    PB_CR1_C11  // Output   ���Ż��̵����������  �ߵ�ƽ��Ч
#define Receiver_LED_TX_CR1     PB_CR1_C12 // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_LED_RX_CR1     PB_CR1_C13  // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_OUT_OPEN_CR1   PD_CR1_C13  // Output   ���Ż��̵���OPEN  �ߵ�ƽ��Ч
#define Receiver_OUT_CLOSE_CR1  PD_CR1_C12  // Output   ���Ż��̵���close  �ߵ�ƽ��Ч
#define Receiver_OUT_STOP_CR1   PD_CR1_C11  // Output   ���Ż��̵���stop  �ߵ�ƽ��Ч
#define Receiver_OUT_VENT_CR1   PD_CR1_C10   // Output ���Ż��̵���VENT  �ߵ�ƽ��Ч
#define Inverters_OUT_CR1       PA_CR1_C12     // ����   �̵�������źŷ���   �͵�ƽ��Ч

#define Receiver_test_CR1       PB_CR1_C10    // Input ���Ż����Խ�  �ߵ�ƽ��Ч


#define ADF7021_DATA_CLK_CR2    PB_CR2_C22     // ����   ����ͬ��ʱ����

/*************************************CMT2310A*****************************************/
#define CMT2310A_GPIO4      PC_IDR_IDR4
#define CMT2310A_GPIO4_DDR  PC_DDR_DDR4
#define CMT2310A_GPIO4_CR1  PC_CR1_C14
#define CMT2310A_GPIO4_CR2  PC_CR2_C24

#define CMT2310A_GPIO5      PC_IDR_IDR3
#define CMT2310A_GPIO5_DDR  PC_DDR_DDR3
#define CMT2310A_GPIO5_CR1  PC_CR1_C13
#define CMT2310A_GPIO5_CR2  PC_CR2_C23

#define CMT2310A_CLK        CMT2310A_GPIO4
#define CMT2310A_DATA       CMT2310A_GPIO5

#define TP3 PA_IDR_IDR4
#define TP3_DDR PA_DDR_DDR4
#define TP3_CR1 PA_CR1_C14
#define TP3_CR2 PA_CR2_C24

#define TP4 PA_IDR_IDR5
#define TP4_DDR PA_DDR_DDR5
#define TP4_CR1 PA_CR1_C15
#define TP4_CR2 PA_CR2_C25

#define CMT2310A_GPIO3              PC_IDR_IDR1
#define CMT2310A_GPIO3_INT1_DDR     PC_DDR_DDR1
#define CMT2310A_GPIO3_INT1_CR1     PC_CR1_C11
#define CMT2310A_GPIO3_INT1_CR2     PC_CR2_C21


/********************�����л� CG2214M6�������żĴ���*****************************************/
#define CG2214M6_VC1 PC_ODR_ODR6     //VC1
#define CG2214M6_VC1_DDR PC_DDR_DDR6 //����
#define CG2214M6_VC1_CR1 PC_CR1_C16  //ģʽMODE
#define CG2214M6_VC1_CR2 PC_CR2_C26  //���Ƶ�ʻ����жϿ���

#define CG2214M6_VC2 PC_ODR_ODR5     //VC2
#define CG2214M6_VC2_DDR PC_DDR_DDR5 //����
#define CG2214M6_VC2_CR1 PC_CR1_C15  //ģʽMODE
#define CG2214M6_VC2_CR2 PC_CR2_C25  //���Ƶ�ʻ����жϿ���

#define CG2214M6_RF1_USE  \
    {                     \
        CG2214M6_VC1 = 0; \
        CG2214M6_VC2 = 1; \
    }
#define CG2214M6_RF2_USE  \
    {                     \
        CG2214M6_VC1 = 1; \
        CG2214M6_VC2 = 0; \
    }
#define CG2214M6_USE_T CG2214M6_RF1_USE
#define CG2214M6_USE_R CG2214M6_RF2_USE




