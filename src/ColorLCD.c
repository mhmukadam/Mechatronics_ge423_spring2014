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
#include "evmomapl138_spi.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "COECSL_edma3.h"
#include "COECSL_registers.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"
#include "ladar.h"

extern EDMA3_CCRL_Regs *EDMA3_0_Regs;
extern bgr *LCD_Image;

unsigned short int *LCD_mem;

int bluegain = 2;
unsigned char bkgrnd_red=0x0F;
unsigned char bkgrnd_green=0x00;
unsigned char bkgrnd_blue=0x00;



void LCD(void) {

	int row;
	int j;
	//num_printLCD++; // for debug
	for(row=7; row<135; row++) {
		for(j=23; j<151; j++){
			// send a pair of pixel data
				LCD_mem[((j-23)/2*3+7)+(row-7)*LCD_MEM_COLS]
					= (((LCD_Image[row*IMAGE_COLUMNS+j].red&0xF0)|(LCD_Image[row*IMAGE_COLUMNS+j].green>>4)) | 0x0100);
				LCD_mem[((j-23)/2*3+8)+(row-7)*LCD_MEM_COLS]
					= ((((LCD_Image[row*IMAGE_COLUMNS+j].blue*bluegain)&0xF0)|(LCD_Image[row*IMAGE_COLUMNS+j+1].red>>4) | 0x0100));
				LCD_mem[((j-23)/2*3+9)+(row-7)*LCD_MEM_COLS]
					= (((LCD_Image[row*IMAGE_COLUMNS+j+1].green&0xF0)|((LCD_Image[row*IMAGE_COLUMNS+j+1].blue*bluegain)>>4)) | 0x0100);
		}
	}


	BCACHE_wb(LCD_mem,2*LCD_MEM_ROWS*LCD_MEM_COLS,EDMA3_CACHE_WAIT);

	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].OPT = OPT_TCINTEN | (EDMA3EVT_SPI1_TRANSMIT<<OPT_TCC_SHIFT);
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].SRC = (uint32_t)&LCD_mem[1];
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].A_B_CNT = ((LCD_MEM_ROWS*LCD_MEM_COLS-1) << 16) | 0x2;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].DST = (uint32_t)&SPI1->SPIDAT0;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].SRC_DST_BIDX = 0x00000002;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].LINK_BCNTRLD = 0x0000FFFF;  // Null Link
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].SRC_DST_CIDX = 0x0;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_SPI1_TRANSMIT].CCNT = 0x1;

		// enable DMA region access SPI1
	EDMA3_0_Regs->DRA[1].DRAE |= (1U<<EDMA3EVT_SPI1_TRANSMIT);
	// clear pending events
	EDMA3_0_Regs->SHADOW[1].ECR = (1U<<EDMA3EVT_SPI1_TRANSMIT);
	EDMA3_0_Regs->SHADOW[1].SECR = (1U<<EDMA3EVT_SPI1_TRANSMIT);
	// enable event interrupt
	EDMA3_0_Regs->SHADOW[1].IESR = (1U<<EDMA3EVT_SPI1_TRANSMIT);
	// enable event
	EDMA3_0_Regs->SHADOW[1].EESR = (1U<<EDMA3EVT_SPI1_TRANSMIT);
	// clear interrupts
	EDMA3_0_Regs->SHADOW[1].ICR = (1U<<EDMA3EVT_SPI1_TRANSMIT);

	SPI1->SPIINT |= (1 << 16);  // enable DMA int request
	SPI1->SPIDAT0 = LCD_mem[0];

	// Old code without EDMA
	/*for(row=0; row<128; row++)
		{
			spi_command(PASET); //command
			spi_data(row+2);
			spi_data(131);
			spi_command(CASET); // command
			spi_data(0);
			spi_data(131);
			spi_command(RAMWR); // command
			for(j=0; j<128; j+=2){
				// send a pair of pixel data
				spi_data(LCD_mem[(j/2)*3+7 + row*LCD_MEM_COLS]);
				spi_data(LCD_mem[(j/2)*3+8 + row*LCD_MEM_COLS]);
				spi_data(LCD_mem[(j/2)*3+9 + row*LCD_MEM_COLS]);
				//spi_data(0xFF);
				//spi_data(0xFF);
				//spi_data(0xFF);
			}
		}*/

		/*if (volup > 0) {
			volume++;
			if (volume == 0x40) volume = 0x00;
			spi_command(VOLCTR);
			spi_data(volume); // volume (6-bit value)
			spi_data(4); // voltage resistor (3-bit value)
			spi_command(PWRCTR);  // power ctrl
			spi_data(0x0F);      //everything on, no external reference resistors
			spi_command(DISON);   // display on
			volup--;
		} else if (volup < 0) {
			volume--;
			if (volume == 0x00) volume = 0x3F;
			spi_command(VOLCTR);
			spi_data(volume); // volume (6-bit value)
			spi_data(4); // voltage resistor (3-bit value)
			spi_command(PWRCTR);  // power ctrl
			spi_data(0x0F);      //everything on, no external reference resistors
			spi_command(DISON);   // display on
			volup++;
		}*/
		//spi_command(VOLUP);

		//GPIO_setOutput(GPIO_BANK0, GPIO_PIN8, OUTPUT_HIGH);
}





void init_LCD_mem(void){
	int row;
	int j;

	LCD_mem = (unsigned short int *)(ADDR_VIDEO_DATA_BASE+LCD_MEM_OFFSET);

	bkgrnd_red=bkgrnd_red&0x0F;
	bkgrnd_blue=bkgrnd_blue&0x0F;
	bkgrnd_green=bkgrnd_green&0x0F;
	for(row=0; row<128; row++)
	{
		LCD_mem[0 + row*LCD_MEM_COLS] = PASET;//PASET; //command - remove masks if not using DMA
		LCD_mem[1 + row*LCD_MEM_COLS] = ((row+2) | 0x0100);
		LCD_mem[2 + row*LCD_MEM_COLS] = (131 | 0x0100);
		LCD_mem[3 + row*LCD_MEM_COLS] = CASET;//CASET; // command
		LCD_mem[4 + row*LCD_MEM_COLS] = (2 | 0x0100);
		LCD_mem[5 + row*LCD_MEM_COLS] = (131 | 0x0100);
		LCD_mem[6 + row*LCD_MEM_COLS] = RAMWR;//RAMWR; // command
		for(j=0; j<128; j++){
			// send a pair of pixel data
			LCD_mem[(j/2)*3+7 + row*LCD_MEM_COLS] = (((bkgrnd_red<<4)|bkgrnd_green) | 0x0100);
			LCD_mem[(j/2)*3+8 + row*LCD_MEM_COLS] = (((bkgrnd_blue<<4)|bkgrnd_red) | 0x0100);
			LCD_mem[(j/2)*3+9 + row*LCD_MEM_COLS] = (((bkgrnd_green<<4)|bkgrnd_blue) | 0x0100);
		}
	}
}

//void LCDClearScreen(void) {
//	long i; // loop counter
//	// Row address set (command 0x2B)
//	spi_command(0x2B);
//	spi_data(0);
//	spi_data(131);
//	// Column address set (command 0x2A)
//	spi_command(0x2A);
//	spi_data(0);
//	spi_data(131);
//	// set the display memory to BLACK
//	spi_command(0x2C);
//	for (i = 0; i < ((131 * 131) / 2)+1; i++) {
//		spi_data((0xFFF >> 4) & 0xFF);
//		spi_data(((0xFFF & 0xF) << 4) | ((0xFFF >> 8) & 0xF));
//		spi_data(0xFFF & 0xFF);
//	}
//}

void init_LCD(void){

	long index = 0;

	SET_LCD_RESET;
	//GPIO_setOutput(GPIO_BANK7, GPIO_PIN8, OUTPUT_LOW);

	for (index=0;index<100000;index++) {}

	CLR_LCD_RESET;

	for (index=0;index<100000;index++) {} //delay

	//GPIO_setOutput(GPIO_BANK7, GPIO_PIN8, OUTPUT_HIGH);
	spi_config_t myspiconfig = {SPI_MODE_MASTER,SPI_4PIN_CS,SPI_CS_ACTIVE_LOW,SPI_SHIFT_MSB,1,0,10*1024*1024};
	SPI_init(SPI1,&myspiconfig);

#ifdef PHILIPSCOLORLCD

	// Philips Controller
	spi_command(SLEEPOUT); //SLEEPOUT
	spi_command(INVOFF); //INVON
	spi_command(COLMOD); //COLMOD
	spi_data(0x03); // 12 bits-per-pixel
	spi_command(MADCTL); // MADCTL
	spi_data(0x00); // mirror x,y and reversge RGB
	spi_command(SETCON); //SETCON
	spi_data(0x3F);

	for (index=0;index<100000;index++) {} //delay

	spi_command(DISPON); //DISPON

#else

	// EPSON Controller
	spi_command(DISCTL);  // display control
	spi_data(0x03);
	spi_data(32);
	spi_data(12);
	spi_data(0x00);

	spi_command(COMSCN);  // comscn
	spi_data(0x01);

	spi_command(OSCON);  // oscon

	spi_command(SLPOUT);  // sleep out

	// set the display volume (~= contrast/brightness)
	spi_command(VOLCTR);
	spi_data(0x2A); // volume (6-bit value)
	spi_data(4); // voltage resistor (3-bit value)


	spi_command(PWRCTR);  // power ctrl
	spi_data(0x0F);      //everything on, no external reference resistors

	spi_command(DISINV);  // inverse display mode

	spi_command(DATCTL);  // datctl
	spi_data(0x00);
	spi_data(0);
	spi_data(0x02); //change to 0x02 to use 4096 colors
	spi_data(0x00);


	spi_command(DISON);   // display on

#endif

	set_background();
}

void spi_data(unsigned char data) {
	unsigned int tmp=data;
	tmp |= 0x100;
	SPI1->SPIDAT0 = tmp;
	// wait for data to arrive.
    while (CHKBIT(SPI1->SPIBUF, RXEMPTY)) {}
}

void spi_command(unsigned char data) {
	unsigned int tmp=data;
	SPI1->SPIDAT0 = tmp;
	// wait for data to arrive.
    while (CHKBIT(SPI1->SPIBUF, RXEMPTY)) {}
}

void set_background(void){

	int i;

	bkgrnd_red=bkgrnd_red&0x0F;
	bkgrnd_blue=bkgrnd_blue&0x0F;
	bkgrnd_green=bkgrnd_green&0x0F;

	spi_command(CASET);   // column start/end ram
	spi_data(0);
	spi_data(131);
	spi_command(PASET);   // page start/end ram
	spi_data(0);            // for some reason starts at 2
	spi_data(131);
	spi_command(RAMWR);

	for (i = 0; i < ((131 * 131) / 2)+1; i++) {

		// send a pair of pixel data
		spi_data((bkgrnd_red<<4)|bkgrnd_green);
		spi_data((bkgrnd_blue<<4)|bkgrnd_red);
		spi_data((bkgrnd_green<<4)|bkgrnd_blue);

	}

}




