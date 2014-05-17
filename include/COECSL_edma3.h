#ifndef _DAN_EDMA3_H_
#define _DAN_EDMA3_H_



/**************************************************************************\
* Register Overlay Structure for DRA
\**************************************************************************/
typedef struct  {
    volatile unsigned int DRAE;
    volatile unsigned int DRAEH;
} EDMA3_CCRL_DraRegs;

/**************************************************************************\
* Register Overlay Structure for QUEEVTENTRY
\**************************************************************************/
typedef struct  {
    volatile unsigned int QUEEVT_ENTRY;
} EDMA3_CCRL_QueevtentryRegs;

/**************************************************************************\
* Register Overlay Structure for SHADOW
\**************************************************************************/
typedef struct  {
    volatile unsigned int ER;
    volatile unsigned int ERH;
    volatile unsigned int ECR;
    volatile unsigned int ECRH;
    volatile unsigned int ESR;
    volatile unsigned int ESRH;
    volatile unsigned int CER;
    volatile unsigned int CERH;
    volatile unsigned int EER;
    volatile unsigned int EERH;
    volatile unsigned int EECR;
    volatile unsigned int EECRH;
    volatile unsigned int EESR;
    volatile unsigned int EESRH;
    volatile unsigned int SER;
    volatile unsigned int SERH;
    volatile unsigned int SECR;
    volatile unsigned int SECRH;
    volatile unsigned char RSVD0[8];
    volatile unsigned int IER;
    volatile unsigned int IERH;
    volatile unsigned int IECR;
    volatile unsigned int IECRH;
    volatile unsigned int IESR;
    volatile unsigned int IESRH;
    volatile unsigned int IPR;
    volatile unsigned int IPRH;
    volatile unsigned int ICR;
    volatile unsigned int ICRH;
    volatile unsigned int IEVAL;
    volatile unsigned char RSVD1[4];
    volatile unsigned int QER;
    volatile unsigned int QEER;
    volatile unsigned int QEECR;
    volatile unsigned int QEESR;
    volatile unsigned int QSER;
    volatile unsigned int QSECR;
    volatile unsigned char RSVD2[360];
} EDMA3_CCRL_ShadowRegs;


/**************************************************************************\
* Register Overlay Structure for PARAMENTRY
\**************************************************************************/
typedef struct  {
    volatile unsigned int OPT;
    volatile unsigned int SRC;
    volatile unsigned int A_B_CNT;
    volatile unsigned int DST;
    volatile unsigned int SRC_DST_BIDX;
    volatile unsigned int LINK_BCNTRLD;
    volatile unsigned int SRC_DST_CIDX;
    volatile unsigned int CCNT;
} EDMA3_CCRL_ParamentryRegs;


/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile unsigned int REV;
    volatile unsigned int CCCFG;
    volatile unsigned char RSVD0[248];
    volatile unsigned int DCHMAP[64];  // Not Used
    volatile unsigned int QCHMAP[8];
    volatile unsigned char RSVD1[32];
    volatile unsigned int DMAQNUM[8];
    volatile unsigned int QDMAQNUM;
    volatile unsigned char RSVD2[28];
    volatile unsigned int QUETCMAP;
    volatile unsigned int QUEPRI;
    volatile unsigned char RSVD3[120];
    volatile unsigned int EMR;
    volatile unsigned int EMRH;
    volatile unsigned int EMCR;
    volatile unsigned int EMCRH;
    volatile unsigned int QEMR;
    volatile unsigned int QEMCR;
    volatile unsigned int CCERR;
    volatile unsigned int CCERRCLR;
    volatile unsigned int EEVAL;
    volatile unsigned char RSVD4[28];
    EDMA3_CCRL_DraRegs DRA[8];
    volatile unsigned int QRAE[8];
    volatile unsigned char RSVD5[96];
    EDMA3_CCRL_QueevtentryRegs QUEEVTENTRY[8][16];
    volatile unsigned int QSTAT[8];
    volatile unsigned int QWMTHRA;
    volatile unsigned int QWMTHRB;
    volatile unsigned char RSVD6[24];
    volatile unsigned int CCSTAT;
    volatile unsigned char RSVD7[188];
    volatile unsigned int AETCTL;
    volatile unsigned int AETSTAT;
    volatile unsigned int AETCMD;
    volatile unsigned char RSVD8[244];
    volatile unsigned int MPFAR;
    volatile unsigned int MPFSR;
    volatile unsigned int MPFCR;
    volatile unsigned int MPPAG;
    volatile unsigned int MPPA[8];
    volatile unsigned char RSVD9[2000];
    volatile unsigned int ER;
    volatile unsigned int ERH;
    volatile unsigned int ECR;
    volatile unsigned int ECRH;
    volatile unsigned int ESR;
    volatile unsigned int ESRH;
    volatile unsigned int CER;
    volatile unsigned int CERH;
    volatile unsigned int EER;
    volatile unsigned int EERH;
    volatile unsigned int EECR;
    volatile unsigned int EECRH;
    volatile unsigned int EESR;
    volatile unsigned int EESRH;
    volatile unsigned int SER;
    volatile unsigned int SERH;
    volatile unsigned int SECR;
    volatile unsigned int SECRH;
    volatile unsigned char RSVD10[8];
    volatile unsigned int IER;
    volatile unsigned int IERH;
    volatile unsigned int IECR;
    volatile unsigned int IECRH;
    volatile unsigned int IESR;
    volatile unsigned int IESRH;
    volatile unsigned int IPR;
    volatile unsigned int IPRH;
    volatile unsigned int ICR;
    volatile unsigned int ICRH;
    volatile unsigned int IEVAL;
    volatile unsigned char RSVD11[4];
    volatile unsigned int QER;
    volatile unsigned int QEER;
    volatile unsigned int QEECR;
    volatile unsigned int QEESR;
    volatile unsigned int QSER;
    volatile unsigned int QSECR;
    volatile unsigned char RSVD12[3944];
    EDMA3_CCRL_ShadowRegs SHADOW[8];
    volatile unsigned char RSVD13[4096];
    EDMA3_CCRL_ParamentryRegs PARAMENTRY[512];
} EDMA3_CCRL_Regs;

#define OPT_ITCCHEN     (0x00800000)   // bit 23
#define OPT_TCCHEN      (0x00400000)   // bit 22
#define OPT_ITCINTEN    (0x00200000)   // bit 21
#define OPT_TCINTEN     (0x00100000)   // bit 20
#define OPT_TCC         (0x0003F000)   // bit 12-17
#define OPT_TCCMODE     (0x00000800)   // bit 11
#define OPT_FWID        (0x00000700)   // bit 8-10
#define OPT_FWID_8      (0x00000000)   // 0
#define OPT_FWID_16     (0x00000100)   // 1h
#define OPT_FWID_32     (0x00000200)   // 2h
#define OPT_FWID_64     (0x00000300)   // 3h
#define OPT_FWID_128    (0x00000400)   // 4h
#define OPT_FWID_256    (0x00000500)   // 5h
#define OPT_STATIC      (0x00000008)   // bit 3
#define OPT_SYNCDIM     (0x00000004)   // bit 2
#define OPT_DAM         (0x00000002)   // bit 1
#define OPT_SAM         (0x00000001)   // bit 0
#define OPT_TCC_SHIFT   12

typedef enum
{
	EDMA3EVT_MCASP0_RECEIVE,
	EDMA3EVT_MCASP0_TRANSMIT,
	EDMA3EVT_MCBSP0_RECEIVE,
	EDMA3EVT_MCBSP0_TRANSMIT,
	EDMA3EVT_MCBSP1_RECEIVE,
	EDMA3EVT_MCBSP1_TRANSMIT,
	EDMA3EVT_GPIO_BANK0,
	EDMA3EVT_GPIO_BANK1,
	EDMA3EVT_UART0_RECEIVE,
	EDMA3EVT_UART0_TRANSMIT,
	EDMA3EVT_TIMER64P0_EVENTOUT12,
	EDMA3EVT_TIMER64P0_EVENTOUT34,
	EDMA3EVT_UART1_RECEIVE,
	EDMA3EVT_UART1_TRANSMIT,
	EDMA3EVT_SPI0_RECEIVE,
	EDMA3EVT_SPI0_TRANSMIT,
	EDMA3EVT_MMCSD_RECEIVE,
	EDMA3EVT_MMCSD_TRANSMIT,
	EDMA3EVT_SPI1_RECEIVE,
	EDMA3EVT_SPI1_TRANSMIT,
	EDMA3EVT_PRU_EVENTOUT6,
	EDMA3EVT_PRU_EVENTOUT7,
	EDMA3EVT_GPIO_BANK2,
	EDMA3EVT_GPIO_BANK3,
	EDMA3EVT_I2C0_RECEIVE,
	EDMA3EVT_I2C0_TRANSMIT,
	EDMA3EVT_I2C1_RECEIVE,
	EDMA3EVT_I2C1_TRANSMIT,
	EDMA3EVT_GPIO_BANK4,
	EDMA3EVT_GPIO_BANK5,
	EDMA3EVT_UART2_RECEIVE,
	EDMA3EVT_UART2_TRANSMIT
} EDMA3_Event_t;

#define EDMA3_EVENTS        32
#define EDMA3_CACHE_WAIT        (1u)

void init_DMA(void);

#endif /* _dan_edma3_h_ */
