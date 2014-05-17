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
#include "c67fastMath.h" // sinsp,cossp, tansp
#include "evmomapl138.h"
#include "evmomapl138_i2c.h"
#include "evmomapl138_timer.h"
#include "evmomapl138_led.h"
#include "evmomapl138_dip.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "evmomapl138_spi.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"

#include "mcbsp_com.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"

extern mcbspL138_com f28335TXData;

int LCDnewMsgLine1 = 0;
int LCDnewMsgLine2 = 0;

void LCDvPrintfLine(unsigned char line, char *format, va_list ap)
{
	char buffer[120];
	int i;

	vsprintf(buffer,format,ap);
	
	if (line == 1) {		
		f28335TXData.data.lcdline1.chararray[0] = 0xFE;
		f28335TXData.data.lcdline1.chararray[1] = 'G';
		f28335TXData.data.lcdline1.chararray[2] = 1;
		f28335TXData.data.lcdline1.chararray[3] = line;
	} else { 
		f28335TXData.data.lcdline2.chararray[0] = 0xFE;
		f28335TXData.data.lcdline2.chararray[1] = 'G';
		f28335TXData.data.lcdline2.chararray[2] = 1;
		f28335TXData.data.lcdline2.chararray[3] = line;
	}
	for (i=4;i<24;i++) {
		if (i >= strlen(buffer)+4) {
			if (line == 1) {
				f28335TXData.data.lcdline1.chararray[i] = ' ';
			} else {
				f28335TXData.data.lcdline2.chararray[i] = ' ';
			}
		} else {
			if (line == 1 ) {
				f28335TXData.data.lcdline1.chararray[i] = buffer[i-4];
			} else {
				f28335TXData.data.lcdline2.chararray[i] = buffer[i-4];
			}
		}
	}
}


void LCDPrintfLine(unsigned char line, char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    LCDvPrintfLine(line,format,ap);
    if (line == 1) {
    	LCDnewMsgLine1 = 1;
    } else {
    	LCDnewMsgLine2 = 1;
    }
}
