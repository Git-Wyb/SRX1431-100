/***********************************************************************/
/*  FILE        :initial.h                                             */
/*  DATE        :Mar, 2014                                             */
/*  Programmer	:xiang 'R                                              */
/*  CPU TYPE    :STM8L151G6     Crystal: 16M HSI                       */
/*  Mark        :ver 1.0                                               */
/***********************************************************************/
/***********************系统预定义**********************/
#define	UINT8	    unsigned char
#define	INT8	    char
#define	UINT16	    unsigned int
#define	INT16	    int
#define	UINT32	    unsigned long
#define	INT32	    long

#define uchar       unsigned char
#define uint        unsigned int
#define ulong       unsigned long

#define _NOP()      asm( " nop " )
#define _EI()       asm( " rim " )
#define _DI()       asm( " sim " )

// 方向控制寄存器1（1=输出／0=输入）
#define Input				0
#define Output				1
/*******************系统预定义  end**********************/

      typedef union
      {
        UINT32  whole_reg;
        UINT8   byte[4];    // Warning: Be endian-specific when accessing bytes

       } ADF70XX_REG_T;

      typedef union {	  // ID No.
	UINT32	IDL ;
	UINT8	IDB[4];
      }uni_rom_id;


void RAM_clean(void);						// 清除RAM
void VHF_GPIO_INIT(void);					// CPU端口设置
void SysClock_Init( void ); 				// 系统时钟（外部时钟）
void beep_init( void );
void Delayus(unsigned char timer);
void RF_test_mode(void );
void WDT_init(void);
void ClearWDT(void);
void CMT2310A_Test_Mode(void);
void RF_BRE_Check(void);
void CG2214M6_GPIO_Init(void);
void BerExtiInit(void);
void BerExtiUnInit(void);
void CMT2310A_GPIO3_INT1_ON(void);
void CMT2310A_GPIO3_INT1_OFF(void);
void CMT2310A_GPIO2_INT2_ON(void);
void CMT2310A_GPIO2_INT2_OFF(void);
