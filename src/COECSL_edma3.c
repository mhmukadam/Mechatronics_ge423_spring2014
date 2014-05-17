#include <std.h>
#include <log.h>
#include <clk.h>
#include <gbl.h>
#include <bcache.h>

#include <mem.h> // MEM_alloc calls
#include <que.h> // QUE functions
#include <sem.h> // Semaphore functions
#include <sys.h>
#include <tsk.h> // TASK functions
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h> // register defines

#include "projectinclude.h"
#include "evmomapl138.h"
#include "COECSL_edma3.h"
#include "evmomapl138_gpio.h"
#include "mcbsp_com.h"
#include "sharedmem.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"
#include "ladar.h"

extern mcbsp28x_com f28335recData;
extern mcbspL138_com f28335TXData;
extern int McBSPrecerror;
extern int firsttimeintMCBSPrec;
extern sharedmemstruct *ptrshrdmem;
extern int LCDnewMsgLine1;
extern int LCDnewMsgLine2;
extern int updateLCD;

extern volatile int LADARready;
extern volatile int LADARprocess;

extern char LADARinBuffer[LADAR_IN_BUFF_SIZE];

EDMA3_CCRL_Regs *EDMA3_0_Regs;
EDMA3_CCRL_Regs *EDMA3_1_Regs;


int f28335RecFirsttime = 1;
float enc1 = 0;
float enc2 = 0;
float enc3 = 0;
float enc4 = 0;
float compass = 0;
float switchstate = -1;
float adcA0 = 0;
float adcB0 = 0;
float adcA1 = 0;
float adcB1 = 0;
float adcA2 = 0;
float adcB2 = 0;
float adcA3 = 0;
float adcB3 = 0;
float adcA4 = 0;
float adcB4 = 0;
float adcA5 = 0;
float adcB5 = 0;
float adcA6 = 0;
float adcA7 = 0;
float F28335_Extra1 = 0;
float F28335_Extra2 = 0;
float F28335_Extra3 = 0;
float F28335_Extra4 = 0;

volatile int new_sendtolinux_F28335_Extra = 0;
volatile float send_F28335_Extra1 = 0;
volatile float send_F28335_Extra2 = 0;
volatile float send_F28335_Extra3 = 0;
volatile float send_F28335_Extra4 = 0;
extern unsigned long timeint;
extern unsigned long mcbspReset;

void init_DMA(void) {

	EDMA3_0_Regs = (EDMA3_CCRL_Regs *)0x01C00000;
	EDMA3_1_Regs = (EDMA3_CCRL_Regs *)0x01E30000;

	EDMA3_1_Regs->QCHMAP[0] = 0x000041C;		// PAENTRY 32  0x1C Trigger so must write to CCNT to start QDMA
	EDMA3_1_Regs->DRA[1].DRAE = 0x80000000; // Enable Event 31 for shadow region 1 for TCC = 31 for QDMA channel 0
	EDMA3_1_Regs->QRAE[1] = 0x1;   // Enable QDMA0 for shadow region 1

	EDMA3_1_Regs->SHADOW[1].QSECR = 0x1;  // Clear any possible secondary events
	EDMA3_1_Regs->SHADOW[1].ICR = 0x80000000;  // Clear ipr bit 31 from QDMA0 interrupt
	EDMA3_1_Regs->SHADOW[1].QEESR = 0x1;  // Enable QDMA channel 0
	EDMA3_1_Regs->SHADOW[1].IESR = 0x80000000;  // Enable interrupt for TCC = 31


	EDMA3_1_Regs->QCHMAP[1] = 0x000043C;		// PAENTRY 33  0x3C Trigger so must write to CCNT to start QDMA
	EDMA3_1_Regs->DRA[1].DRAE |= 0x40000000; // Enable Event 30 for shadow region 1 for TCC = 30 for QDMA channel 0
	EDMA3_1_Regs->QRAE[1] |= 0x2;   // Enable QDMA0 for shadow region 1 - Don't need?

	EDMA3_1_Regs->SHADOW[1].QSECR |= 0x2;  // Clear any possible secondary events
	EDMA3_1_Regs->SHADOW[1].ICR |= 0x40000000;  // Clear ipr bit 30 from QDMA1 interrupt
	EDMA3_1_Regs->SHADOW[1].QEESR |= 0x2;  // Enable QDMA channel 1
	EDMA3_1_Regs->SHADOW[1].IESR |= 0x40000000;  // Enable interrupt for TCC = 30

	// EDMA3_1_CC0_INT1 (91) interrupt serviced by INT5
	ICR = 0x20; // clear pending interrupts
	IER |= 0x20; // enable interrupt on line
}



int num_EDMA3_0 = 0;
float enc0_array[1000];

void EDMA3_0_ISR(void) {

//    now do something with data.

	if ((EDMA3_0_Regs->SHADOW[1].IPR & (1U<<EDMA3EVT_MCBSP1_RECEIVE)) > 0){

		if (firsttimeintMCBSPrec == 0) {

			XBUF1 = 0;  // for some reason here I need to send one additional 32bits to get the communication rolling
			firsttimeintMCBSPrec = 1;

		}

		if ( f28335recData.data.validcode == VALID_28XTO67X	) {

			mcbspReset = timeint;

			enc1 = f28335recData.data.SPI_ENC[0];  // left/right motors in radians
			enc2 = f28335recData.data.SPI_ENC[1];
			enc3 = f28335recData.data.SPI_ENC[2];
			enc4 = f28335recData.data.SPI_ENC[3];
			compass = f28335recData.data.compass;
			switchstate = f28335recData.data.switchstate;
			adcA0 = f28335recData.data.ADC[0];
			adcB0 = f28335recData.data.ADC[1];
			adcA1 = f28335recData.data.ADC[2];
			adcB1 = f28335recData.data.ADC[3];
			adcA2 = f28335recData.data.ADC[4];
			adcB2 = f28335recData.data.ADC[5];
			adcA3 = f28335recData.data.ADC[6];
			adcB3 = f28335recData.data.ADC[7];
			adcA4 = f28335recData.data.ADC[8];
			adcB4 = f28335recData.data.ADC[9];
			adcA5 = f28335recData.data.ADC[10];
			adcB5 = f28335recData.data.ADC[11];
			adcA6 = f28335recData.data.ADC[12];
			adcA7 = f28335recData.data.ADC[13];
			F28335_Extra1 = f28335recData.data.F28335_EXTRA[0];
			F28335_Extra2 = f28335recData.data.F28335_EXTRA[1];
			F28335_Extra3 = f28335recData.data.F28335_EXTRA[2];
			F28335_Extra4 = f28335recData.data.F28335_EXTRA[3];
			if (new_sendtolinux_F28335_Extra == 0) {
				send_F28335_Extra1 = F28335_Extra1;
				send_F28335_Extra2 = F28335_Extra2;
				send_F28335_Extra3 = F28335_Extra3;
				send_F28335_Extra4 = F28335_Extra4;
				new_sendtolinux_F28335_Extra = 1;
			}
		} else {
			McBSPrecerror++;
		}

		if (f28335RecFirsttime == 1) {
			f28335RecFirsttime = 0;
		} else {
			f28335TXData.data.flags = 0;
		}

		//call swi
		SWI_post(&SWI_RobotControl);

		if (num_EDMA3_0 < 1000) {
			enc0_array[num_EDMA3_0] = f28335recData.data.SPI_ENC[0];
		} else {
			num_EDMA3_0 = 0;
			enc0_array[num_EDMA3_0] = f28335recData.data.SPI_ENC[0];
		}

		//f28335TXData.data.flags = FLAGL138_PICONTRL_MODE_BIT0;

		//lcd stuff was here

		//f28335TXData.data.PWM[0] = f28335recData.data.SPI_ENC[0];


		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].OPT = OPT_TCINTEN | (EDMA3EVT_MCBSP1_RECEIVE<<OPT_TCC_SHIFT) | OPT_SYNCDIM;
		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].SRC = RBUF1_ADDRESS;
		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].A_B_CNT = 0x4 | (NUM_28XCOMWORDS<<16);  //0x00060004;  // recieve 6 32bit words or 24 bytes A B synchronise
		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].DST = (uint32_t)&f28335recData.darray[0];
		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].SRC_DST_BIDX = 0x00040000;  // Do not increment SRC but increment DST by 4 bytes
		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].LINK_BCNTRLD = 0x0000FFFF;  // Null Link
		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].SRC_DST_CIDX = 0x0;
		EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].CCNT = 0x1;

		BCACHE_inv(f28335recData.darray, sizeof(mcbsp28x_com), 1);


		EDMA3_0_Regs->SHADOW[1].ICR = (1U<<EDMA3EVT_MCBSP1_RECEIVE);

	} else if ((EDMA3_0_Regs->SHADOW[1].IPR & (1U<<EDMA3EVT_SPI1_TRANSMIT)) > 0){
		num_EDMA3_0++;
		updateLCD = 1;
		EDMA3_0_Regs->SHADOW[1].ICR = (1U<<EDMA3EVT_SPI1_TRANSMIT);
	} else if ((EDMA3_0_Regs->SHADOW[1].IPR & (1U<<EDMA3EVT_UART1_RECEIVE)) > 0){
		LADARprocess = 1;

		EDMA3_0_Regs->SHADOW[1].ICR = (1U<<EDMA3EVT_UART1_RECEIVE);
	}

	// check IPR for other pending EDMA3_1 interrupts
	if ((EDMA3_0_Regs->SHADOW[1].IPR) > 0){
		EDMA3_0_Regs->SHADOW[1].IEVAL = 0x00000001;
	}

}

int numofints = 0;
void EDMA3_1_ISR(void) {
	numofints++;


	if ((EDMA3_1_Regs->SHADOW[1].IPR & 0x80000000) > 0){
	//	I going to be using EDMA3 number 1 EDMA3_1_Regs
	//	EDMA3_1_CC0_INT1 = 91

		//GPIO_setOutput(IMAGE_TO_LINUX_BANK, IMAGE_TO_LINUX_FLAG, OUTPUT_LOW);
		CLR_IMAGE_TO_LINUX;

		// Flush or write back source
	    BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

		EDMA3_1_Regs->SHADOW[1].ICR = 0x80000000;  // Clear ipr bit 31 from QDMA0 interrupt

	} else if ((EDMA3_1_Regs->SHADOW[1].IPR & 0x40000000) > 0){

		// post SWI to format data for color LCD
		SWI_post(&SWI_LCD);
		EDMA3_1_Regs->SHADOW[1].ICR = 0x40000000;  // Clear ipr bit 31 from QDMA0 interrupt
	}

	// check IPR for other pending EDMA3_1 interrupts
	if ((EDMA3_1_Regs->SHADOW[1].IPR) > 0){
		EDMA3_1_Regs->SHADOW[1].IEVAL = 0x00000001;
	}

}
