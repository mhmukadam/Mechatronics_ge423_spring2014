#ifndef _COECSL_REGISTERS_H_
#define _COECSL_REGISTERS_H_

#define TIMER1_BASE			0x01C21000
#define T1_TIM12              *(volatile unsigned int*)(TIMER1_BASE + 0x010)
#define T1_TIM34              *(volatile unsigned int*)(TIMER1_BASE + 0x014)
#define T1_TCR                *(volatile unsigned int*)(TIMER1_BASE + 0x020)
#define T1_TGCR               *(volatile unsigned int*)(TIMER1_BASE + 0x024)
#define TGCR_TIMMODE_32BIT_UNCHAINED  0x4
#define TGCR_TIM12_RESET	0x1
#define TGCR_TIM34_RESET	0x2

#define SYSCONFIG1_REG_BASE    (0x01E2C000)
#define SYSCONFIG1          ((sysconfig1_regs_t *)SYSCONFIG1_REG_BASE)


/*
Five LEDS
								GPIO6.8
								GPIO6.9
								GPIO6.10
								GPIO6.11
								GPIO6.12

Maybe used by USB				GPIO6.13

COLOR LCD RESET  				GPIO7.8
RED LED  						GPIO7.9
BLUE LED 						GPI07.10
GREEN LED 						GPIO7.11
LCD Pushbutton1  				GPIO7.12
LCD Pushbutton2  				GPIO7.13

Flags:
DATA_FROM_LINUX 				GPIO0.0
DATA_TO_LINUX 					GPIO0.1
VBDATA_FROM_LINUX				GPIO0.2
VBDATA_TO_LINUX					GPIO0.3

currently I set GPIO0.0 to 6 as GPIO inputs first and then I set them to McBSP1 pins after linux has booted.  

LINUX_BOOTED					GPIO1.14
IMAGE_TO_LINUX 					GPIO1.2
OPTITRACKDATA_FROM_LINUX		GPIO1.3
DATAFORFILE_TO_LINUX 			GPIO1.4


Unused and can be used for flags:
GPIO0.4
GPIO0.5
GPIO0.6

GPIO1.0
GPIO1.1
GPIO1.5
GPIO1.8
GPIO1.9
GPIO1.10
GPIO1.11
GPIO1.12
GPIO1.13

There are more unused on GPIO4 but I don't think we will need them.

*/



#define OUTGPIO67              *(volatile unsigned int*)(0x01e2608c)
#define SETGPIO67              *(volatile unsigned int*)(0x01e26090)
#define CLRGPIO67              *(volatile unsigned int*)(0x01e26094)
#define INGPIO67               *(volatile unsigned int*)(0x01E26098)

#define OUTGPIO01			   *(volatile unsigned int*)(0x01E26014)
#define SETGPIO01			   *(volatile unsigned int*)(0x01E26018)
#define CLRGPIO01			   *(volatile unsigned int*)(0x01E2601c)
#define INGPIO01               *(volatile unsigned int*)(0x01E26020)

#define SETLED1 (SETGPIO67 = 0x00000100)
#define SETLED2 (SETGPIO67 = 0x00000200)
#define SETLED3 (SETGPIO67 = 0x00000400)
#define SETLED4 (SETGPIO67 = 0x00000800)
#define SETLED5 (SETGPIO67 = 0x00001000)

#define CLRLED1 (CLRGPIO67 = 0x00000100)
#define CLRLED2 (CLRGPIO67 = 0x00000200)
#define CLRLED3 (CLRGPIO67 = 0x00000400)
#define CLRLED4 (CLRGPIO67 = 0x00000800)
#define CLRLED5 (CLRGPIO67 = 0x00001000)

#define GETLED1STATE (((OUTGPIO67 & 0x00000100)>>8) & 0x00000001)
#define GETLED2STATE (((OUTGPIO67 & 0x00000200)>>9) & 0x00000001)
#define GETLED3STATE (((OUTGPIO67 & 0x00000400)>>10) & 0x00000001)
#define GETLED4STATE (((OUTGPIO67 & 0x00000800)>>11) & 0x00000001)
#define GETLED5STATE (((OUTGPIO67 & 0x00001000)>>12) & 0x00000001)


#define SETREDLED (CLRGPIO67 = 0x02000000)
#define SETBLUELED (CLRGPIO67 = 0x04000000)
#define SETGREENLED (CLRGPIO67 = 0x08000000)

#define CLRREDLED (SETGPIO67 = 0x02000000)
#define CLRBLUELED (SETGPIO67 = 0x04000000)
#define CLRGREENLED (SETGPIO67 = 0x08000000)

#define GETSWITCH1 (((INGPIO67 & 0x10000000)>>28) & 0x00000001)
#define GETSWITCH2 (((INGPIO67 & 0x20000000)>>29) & 0x00000001)

#define SET_OPTITRACKDATA_FROM_LINUX (SETGPIO01 = 0x00080000)
#define CLR_OPTITRACKDATA_FROM_LINUX (CLRGPIO01 = 0x00080000)
#define GET_OPTITRACKDATA_FROM_LINUX ((OUTGPIO01 & 0x00080000)>>19)

#define SET_DATA_FROM_LINUX (SETGPIO01 = 0x00000001)
#define CLR_DATA_FROM_LINUX (CLRGPIO01 = 0x00000001)
#define GET_DATA_FROM_LINUX (OUTGPIO01 & 0x00000001)

#define SET_VBDATA_FROM_LINUX (SETGPIO01 = 0x00000004)
#define CLR_VBDATA_FROM_LINUX (CLRGPIO01 = 0x00000004)
#define GET_VBDATA_FROM_LINUX ((OUTGPIO01 & 0x00000004)>>2)

#define SET_VBDATA_TO_LINUX (SETGPIO01 = 0x00000008)
#define CLR_VBDATA_TO_LINUX (CLRGPIO01 = 0x00000008)
#define GET_VBDATA_TO_LINUX ((OUTGPIO01 & 0x00000008)>>3)

#define SET_DATA_TO_LINUX (SETGPIO01 = 0x00000002)
#define CLR_DATA_TO_LINUX (CLRGPIO01 = 0x00000002)
#define GET_DATA_TO_LINUX ((OUTGPIO01 & 0x00000002)>>1)

#define SET_IMAGE_TO_LINUX (SETGPIO01 = 0x00040000)
#define CLR_IMAGE_TO_LINUX (CLRGPIO01 = 0x00040000)
#define GET_IMAGE_TO_LINUX ((OUTGPIO01 & 0x00040000)>>18)

#define SET_LCD_RESET (CLRGPIO67 = 0x01000000)
#define CLR_LCD_RESET (SETGPIO67 = 0x01000000)

#define GET_ISLINUX_BOOTED ((OUTGPIO01 & 0x40000000)>>30)
#define SET_ISLINUX_BOOTED (SETGPIO01 = 0x40000000)
#define CLR_ISLINUX_BOOTED (CLRGPIO01 = 0x40000000)

#define GET_DATAFORFILE_TO_LINUX ((OUTGPIO01 & 0x00100000)>>20)
#define SET_DATAFORFILE_TO_LINUX (SETGPIO01 = 0x00100000)
#define CLR_DATAFORFILE_TO_LINUX (CLRGPIO01 = 0x00100000)

// Not needed any more Old name - remove these defines after SP13 semester
//#define SET_CONTINUOUSDATA_TO_LINUX (SETGPIO01 = 0x00080000)
//#define CLR_CONTINUOUSDATA_TO_LINUX (CLRGPIO01 = 0x00080000)
//#define GET_CONTINUOUSDATA_TO_LINUX ((OUTGPIO01 & 0x00080000)>>19)

typedef struct
{
   volatile uint32_t VTPIO_CTL;
   volatile uint32_t DDR_SLEW;
   volatile uint32_t DEEP_SLEEP;
   volatile uint32_t PUPD_ENA;
   volatile uint32_t PUPD_SEL;
   volatile uint32_t RXACTIVE;
   volatile uint32_t PWRDN;
} sysconfig1_regs_t;



#endif /* _COECSL_REGISTERS_H_ */
