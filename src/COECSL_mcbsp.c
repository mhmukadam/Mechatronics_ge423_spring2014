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
#include "mcbsp_com.h"
#include "COECSL_mcbsp.h"
#include "COECSL_edma3.h"


unsigned long servosPosted = 0;
int newServos = 0;

extern int LCDnewMsgLine1;
extern int LCDnewMsgLine2;

extern EDMA3_CCRL_Regs *EDMA3_0_Regs;

mcbsp28x_com f28335recData;
#pragma DATA_ALIGN(f28335recData,128)
mcbspL138_com f28335TXData;
#pragma DATA_ALIGN(f28335TXData,128)  // not sure if this is necessary - added to be careful with invalidating cache

int McBSPrecerror = 0;
int firsttimeintMCBSPrec = 0;

unsigned long timeint = 0;
unsigned long mcbspReset = 0;

int testcount = 0;

void init_McBSP(void) {

	volatile uint32_t index;
	// Initial McBSP setup before taking out of reset
	//	DRR1  Data Recieve Register
	// DXR1  Data Transmit Register
	SPCR1 = 0x00000000;  // serial port in reset
	WFIFOCTL1 = 0;
	RFIFOCTL1 = 0;

	WFIFOCTL1 = 0x0101;  // WNUMEVT = 1, WNUMDMA = 1
	WFIFOCTL1 |= 0x10000; // Enable McBSP Write FIFO
//	RFIFOCTL1 = 0x0601;  // RNUMEVT = 6, RNUMDMA = 1
	RFIFOCTL1 = (NUM_28XCOMWORDS << 8) | 0x01;  // RNUMEVT = 6, RNUMDMA = 1
	RFIFOCTL1 |= 0x10000; // Enable McBSP Read FIFO


    BCACHE_inv(f28335recData.darray, sizeof(mcbsp28x_com), 1);

	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].OPT = OPT_TCINTEN | (EDMA3EVT_MCBSP1_RECEIVE<<OPT_TCC_SHIFT) | OPT_SYNCDIM;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].SRC = RBUF1_ADDRESS;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].A_B_CNT = 0x4 | (NUM_28XCOMWORDS<<16);  //0x00060004;  // recieve 6 32bit words or 24 bytes A B synchronise
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].DST = (uint32_t)&f28335recData.darray[0];
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].SRC_DST_BIDX = 0x00040000;  // Do not increment SRC but increment DST by 4 bytes
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].LINK_BCNTRLD = 0x0000FFFF;  // Null Link
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].SRC_DST_CIDX = 0x0;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_MCBSP1_RECEIVE].CCNT = 0x1;

		// enable DMA region access MCBSP1
	EDMA3_0_Regs->DRA[1].DRAE |= (1U<<EDMA3EVT_MCBSP1_RECEIVE);
	// clear pending events
	EDMA3_0_Regs->SHADOW[1].ECR = (1U<<EDMA3EVT_MCBSP1_RECEIVE);
	EDMA3_0_Regs->SHADOW[1].SECR = (1U<<EDMA3EVT_MCBSP1_RECEIVE);
	// enable event interrupt
	EDMA3_0_Regs->SHADOW[1].IESR = (1U<<EDMA3EVT_MCBSP1_RECEIVE);
	// enable event
	EDMA3_0_Regs->SHADOW[1].EESR = (1U<<EDMA3EVT_MCBSP1_RECEIVE);
	// clear interrupts
	EDMA3_0_Regs->SHADOW[1].ICR = (1U<<EDMA3EVT_MCBSP1_RECEIVE);


	RCR1 = 0x000100a0;
	XCR1 = 0x000100a0;
	SRGR1 = 0x2000000E;  // clock divide to 15 with 150MHz clk gives 10Mhz clk
	MCR1 = 0x0;
	PCR1 = 0x00000a00;
	RCERE1_0 = 0x0;
	XCERE1_0 = 0x0;
	RCERE1_1 = 0x0;
	XCERE1_1 = 0x0;
	RCERE1_2 = 0x0;
	XCERE1_2 = 0x0;
	RCERE1_3 = 0x0;
	XCERE1_3 = 0x0;

	// Small delay for internal McBSP synchronization
	for (index=0;index<100000;index++) {}  // probably too long of a wait??

	// Take sample rate generator out of reset
	SPCR1 |= 0x00400000;

	// Small delay for internal McBSP synchronization
	for (index=0;index<100000;index++) {}  // probably too long of a wait??

	// Take transmit out of reset and clear possible XSYNCERRs
	SPCR1 |= 0x00010000;

	// Small delay to waitfor any erroronous sync errors to occur
	for (index=0;index<100000;index++) {}  // probably too long of a wait??

	// Disable transmit again to clear any possible sync errors
	SPCR1 &= ~(0x00010000);

	// Small delay
	for (index=0;index<100000;index++) {}  // probably too long of a wait??

	// *********************************************
	// here if useing EDMA I should set it up here
	// *********************************************

	// Take transmit and recieve out of reset
	SPCR1 |= 0x00010001;

	// EDMA3_0_CC0_INT1 (8) interrupt serviced by INT8
	ICR = 0x100; // clear pending interrupts
	IER |= 0x100; // enable interrupt on line
}


int danindex = 0;
void startMcBSP(void) {

	if (firsttimeintMCBSPrec == 0) {
		f28335TXData.data.PWM[0] = 25.0;
		f28335TXData.data.PWM[1] = 8.9;
//		for (danindex=0;danindex<NUM_L138COMWORDS;danindex++) {

			XBUF1 = f28335TXData.darray[danindex];
//		}
		mcbspReset = timeint;
	} else {
		// if last time received data was long ago then
		if ((timeint - mcbspReset)>100) {
			init_McBSP();
			firsttimeintMCBSPrec = 0;
			testcount++;
		}
	}
}

void addtime(void){
	timeint++;
}


void send_McBSP(void) {
	int i = 0;
	f28335TXData.data.validcode = VALID_67XTO28X;
	if ((WFIFOSTS1 & 0xFF) == 0) {  // if last write transfer is finished
		for (i=0;i<NUM_L138COMWORDS;i++) {
			XBUF1 = f28335TXData.darray[i];
		}
	}
}

int firsttimeLine1 = 1;
int firsttimeLine2 = 1;
void SetRobotOutputs(float vref, float turn, float pwm3, float pwm4, float pwm5, float pwm3b, float pwm4b, float reserved, float dac1, float dac2) {
	if (LCDnewMsgLine1 == 1) {
		f28335TXData.data.flags |= FLAGL138_NEWLCDLINE1_BIT1;
		if (!firsttimeLine1) {
			LCDnewMsgLine1 = 0;
		} else {
			firsttimeLine1 = 0;
		}
	}
	if (LCDnewMsgLine2 == 1) {
		f28335TXData.data.flags |= FLAGL138_NEWLCDLINE2_BIT2;
		if (!firsttimeLine2) {
			LCDnewMsgLine2 = 0;
		} else {
			firsttimeLine2 = 0;
		}
	}

	if ((CLK_getltime() - servosPosted) > 55) {
		servosPosted = CLK_getltime();
		f28335TXData.data.flags |= FLAGL138_NEWSERVOS;
	}
	
	f28335TXData.data.flags |= FLAGL138_PICONTRL_MODE_BIT0;
	f28335TXData.data.vref = vref;
	f28335TXData.data.turn = turn;
	f28335TXData.data.PWM[2] = pwm3;
	f28335TXData.data.PWM[3] = pwm3b;
	f28335TXData.data.PWM[4] = pwm4;
	f28335TXData.data.PWM[5] = pwm4b;
	f28335TXData.data.PWM[6] = pwm5;
	f28335TXData.data.PWM[7] = reserved;
	f28335TXData.data.DAC[0] = dac1;
	f28335TXData.data.DAC[1] = dac2;

	send_McBSP();
}

void SetRobotOutputs_manual(float pwm1, float pwm2, float pwm3, float pwm4, float pwm5, float pwm3b, float pwm4b, float reserved, float dac1, float dac2) {
	if (LCDnewMsgLine1 == 1) {
		f28335TXData.data.flags |= FLAGL138_NEWLCDLINE1_BIT1;
		LCDnewMsgLine1 = 0;
	}
	if (LCDnewMsgLine2 == 1) {
		f28335TXData.data.flags |= FLAGL138_NEWLCDLINE2_BIT2;
		LCDnewMsgLine2 = 0;
	}
	if (newServos == 1) {
		f28335TXData.data.flags |= FLAGL138_NEWSERVOS;
		newServos = 0;
	}
	
	//f28335TXData.data.flags &= ~FLAGL138_PICONTRL_MODE_BIT0;
	f28335TXData.data.PWM[0] = pwm1;
	f28335TXData.data.PWM[1] = pwm2;
	f28335TXData.data.PWM[2] = pwm3;
	f28335TXData.data.PWM[3] = pwm3b;
	f28335TXData.data.PWM[4] = pwm4;
	f28335TXData.data.PWM[5] = pwm4b;
	f28335TXData.data.PWM[6] = pwm5;
	f28335TXData.data.PWM[7] = reserved;
	f28335TXData.data.DAC[0] = dac1;
	f28335TXData.data.DAC[1] = dac2;


	send_McBSP();
}


