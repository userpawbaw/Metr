/* C6701.h */

// EXINTF
#define GBLCTL				(volatile unsigned int*)0x01800000
#define CECTL0				(volatile unsigned int*)0x01800008
#define CECTL1				(volatile unsigned int*)0x01800004
#define CECTL2				(volatile unsigned int*)0x01800010
#define CECTL3				(volatile unsigned int*)0x01800014
#define SDCTL				(volatile unsigned int*)0x01800018
#define SDTIM				(volatile unsigned int*)0x0180001C

// TIMER0
#define T0CTL				(volatile unsigned int*)0x01940000
#define T0PRD				(volatile unsigned int*)0x01940004
#define T0CNT				(volatile unsigned int*)0x01940008

// TIMER1
#define T1CTL				(volatile unsigned int*)0x01980000
#define T1PRD				(volatile unsigned int*)0x01980004
#define T1CNT				(volatile unsigned int*)0x01980008

// UARTA
//#define txrx_divl			(volatile unsigned int*)0x02000000
//#define inten_divh		(volatile unsigned int*)0x02000004
//#define intid_fifo		(volatile unsigned int*)0x02000008
//#define linecon			(volatile unsigned int*)0x0200000C
//#define modemcon			(volatile unsigned int*)0x02000010
//#define linestatus		(volatile unsigned int*)0x02000014
//#define modemstatus		(volatile unsigned int*)0x02000018
//#define scratch			(volatile unsigned int*)0x0200001C

// DSP => FPGA 
#define FPGABASE			0x02000000
#define ADDR_SHIFT(A)		(volatile unsigned int*)(FPGABASE + (A << 2))

// UARTA
#define txrx_divl			ADDR_SHIFT(0x000)
#define inten_divh			ADDR_SHIFT(0x001)
#define intid_fifo			ADDR_SHIFT(0x002)
#define linecon				ADDR_SHIFT(0x003)
#define modemcon			ADDR_SHIFT(0x004)
#define linestatus			ADDR_SHIFT(0x005)
#define modemstatus			ADDR_SHIFT(0x006)
#define scratch				ADDR_SHIFT(0x007)

#define DOUT0				ADDR_SHIFT(0x010)
#define DOUT1				ADDR_SHIFT(0x011)
#define DOUT2				ADDR_SHIFT(0x012)

#define FPGALED				ADDR_SHIFT(0x013)

#define DC_DRV_EN			ADDR_SHIFT(0x020)
#define DC_PWM_R			ADDR_SHIFT(0x021)
#define DC_PWM_L			ADDR_SHIFT(0x022)
#define DC_ENCCNT_CLR		ADDR_SHIFT(0x023)
#define DC_ENCCNT_MODE		ADDR_SHIFT(0x024)

#define DC_ENCST_R			ADDR_SHIFT(0x028)
#define DC_ENCST_L			ADDR_SHIFT(0x029)
#define DC_ENC_POSCNT_R		ADDR_SHIFT(0x02A)
#define DC_ENC_POSCNT_L		ADDR_SHIFT(0x02B)
#define DC_ENC_VELCNT_R		ADDR_SHIFT(0x02C)
#define DC_ENC_VELCNT_L		ADDR_SHIFT(0x02D)

#define STEP_PHASE_R		ADDR_SHIFT(0x030)
#define STEP_PHASE_L		ADDR_SHIFT(0x031)

#define BLDC_DRV_MODE		ADDR_SHIFT(0x040)
#define BLDC_DRV_EN			ADDR_SHIFT(0x041)
#define BLDC_PWM0			ADDR_SHIFT(0x042)
#define BLDC_PWM1			ADDR_SHIFT(0x043)
#define BLDC_PWM2			ADDR_SHIFT(0x044)
#define BLDC_HALLCNT_CLR	ADDR_SHIFT(0x045)
#define BLDC_DRIVE			ADDR_SHIFT(0x046)

#define BLDC_HALLST			ADDR_SHIFT(0x048)
#define BLDC_POSCNT			ADDR_SHIFT(0x049)
#define BLDC_VELCNT			ADDR_SHIFT(0x04A)

#define MUX_SEL				ADDR_SHIFT(0x050)
#define ADCIN				ADDR_SHIFT(0x051)

#define DAC0				ADDR_SHIFT(0x058)
#define DAC1				ADDR_SHIFT(0x059)
#define DAC2				ADDR_SHIFT(0x05A)
#define DAC3				ADDR_SHIFT(0x05B)
#define DAC4				ADDR_SHIFT(0x05C)
#define DAC5				ADDR_SHIFT(0x05D)
#define DAC6				ADDR_SHIFT(0x05E)
#define DAC7				ADDR_SHIFT(0x05F)

#define FPGAVER				ADDR_SHIFT(0x0FF)



