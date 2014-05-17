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

#include "tistdtypes.h"
#include "projectinclude.h"
#include "c67fastMath.h" // sinsp,cossp, tansp
#include "evmomapl138.h"
#include "evmomapl138_i2c.h"
#include "evmomapl138_timer.h"
#include "evmomapl138_led.h"
#include "evmomapl138_dip.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"
#include "pru.h"
#include "PRUcode\rgb_final_bin.h"
#include "mcbsp_com.h"
#include "ColorVision.h"
#include "sharedmem.h"
#include "ladar.h"

int updateLCD = 1;
uint8_t testvar = 0;

int *pic_data;

bgr *Image_data;
bgr *Linux_Image;
bgr *LCD_Image;

unsigned char i2cdata[40];

#define MAX_NUM_EQUIVALENCIES 256
far int equivalency_objects[MAX_NUM_EQUIVALENCIES];  // An array noting equivalency between object labels
far int temp_equivalency_objects[MAX_NUM_EQUIVALENCIES];  // An array noting equivalency between object labels

far struct object_stats{
	int sum_r;
	int sum_c;
	int num_pixels_in_object;
} object_stats[MAX_NUM_EQUIVALENCIES];

#define MAX_NUM_OBJECTS 20
struct final_object_stats{
	int sum_r;
	int sum_c;
	float C11_sum;
	float C20_sum;
	float C02_sum;
	int num_pixels_in_object;
	float center_r;
	float center_c;
	float center_x;
	float center_y;
	float theta;

} final_object_stats[MAX_NUM_OBJECTS+1];

typedef struct hsvtag {
	unsigned char h;
	unsigned char s;
	unsigned char v;
} hsv;

// Image variables can be used to pass information to PRD
volatile int noimagefound = 1;
volatile float 	object_x 			= 0.0;
volatile float 	object_y 			= 0.0;
volatile int 	new_coordata 		= 0;
volatile int numpels = 0;
volatile int new_num_found_objects = 0;
volatile float   new_object_theta 	= 0.0;
volatile float   new_C20				= 0.0;
volatile float   new_C02				= 0.0;


int k;
int r, c; // rows and columns
int top = 0;
int left = 0;
int num_unique_objects = 0;
int current_object = 0;

int color = 0;
int color2 = 100;

float Cdifference = 0.0;

int rprime;
int cprime;

int rbar;
int cbar;
int neighbor_type = 0;
int too_many_objects = 0;
int object_detected = 0;
int largest_num_pixels;
int largest_object;
int red,green,blue,hue,sat,value;
int min,delta;

unsigned char print_hue= 0,print_sat = 0,print_val = 0;
unsigned char print_r = 0,print_b=0,print_g=0;

int specs_s = 0;
int specs_srad = 0;
int specs_v = 0;
int specs_vrad = 0;
int specs_h = 0;
int specs_h2 = 0;
int specs_hrad = 0;

unsigned char *Thres_Image;

sharedmemstruct *ptrshrdmem;

extern EDMA3_CCRL_Regs *EDMA3_1_Regs;
extern float switchstate;

int Check_Equivalency(int A, int B);
int Set_Equivalency (int A, int B);
int Fix_Equivalency(int num_equivalencies_used);

int my_min(int a, int b, int c) {
	int min;
	
	min = a;
	if (b < min) {
		min = b;
	}
	if (c < min) {
			min = c;
	}
	return (min);
		
}

int my_max(int a, int b, int c) {
	int max;
	
	max = a;
	if (b > max) {
		max = b;
	}
	if (c > max) {
			max = c;
	}
	return (max);
		
}

//***********************************************************************************//
volatile float 	bobject_x = 0.0, oobject_x = 0.0;
volatile float 	bobject_y = 0.0, oobject_y = 0.0;
volatile int bnumpels = 0, onumpels = 0;

char colortoggle = 0; // 0 for blue, 1 for orange


//***********************************************************************************//
void init_ColorVision(void) {
	
	// initialize ov6620 cmos camera
	i2cdata[0] = 0x01;
	i2cdata[1] = 0x8F;  // Blue Gain control (default 0x80)
	i2cdata[2] = 0x8F;  // Red Gain Control
	i2cdata[3] = 0x80;  // Saturation
	i2cdata[4] = 0x00;  // Reserved
	i2cdata[5] = 0x4f;  // Contrast
	i2cdata[6] = 0x9f; // Brightness
	i2cdata[7] = 0xCF; // Sharpness  (default 0xC6) 			
	i2cdata[8] = 0x00; // Reserved			
	i2cdata[9] = 0x00; // Reserved   		
	i2cdata[10] = 0x00; // Reserved   		
	i2cdata[11] = 0x00; // Reserved   	
	i2cdata[12] = 0x20; // AWB - Blue   		
	i2cdata[13] = 0x20; // AWB - Red   		
	i2cdata[14] = 0x0D; // COMR   		
	i2cdata[15] = 0x05; // COMS   		
	i2cdata[16] = 0x9A; // AEC
	i2cdata[17] = 0x01; // CLKRC   		
	i2cdata[18] = 0x28; // COMA  
	i2cdata[19] = 0x01; // 0x01; // COMB  
	I2C_write(I2C0, 0x60, i2cdata, 20, SET_STOP_BIT_AFTER_WRITE);
	
	i2cdata[0] = 0x20;
	i2cdata[1] = 0x01; // COME
	I2C_write(I2C0, 0x60, i2cdata, 2, SET_STOP_BIT_AFTER_WRITE);
	
	i2cdata[0] = 0x28;
	i2cdata[1] = 0x81; // COMH
	I2C_write(I2C0, 0x60, i2cdata, 2, SET_STOP_BIT_AFTER_WRITE);
	
	i2cdata[0] = 0x39;
	// changed to PCLK always on.  
	i2cdata[1] = 0x00; //0x40; // COML
	I2C_write(I2C0, 0x60, i2cdata, 2, SET_STOP_BIT_AFTER_WRITE);
	
		
    VPIF_initReceive(VIDEO_CONN_CAMERA);

	// PRU setup
	EVMOMAPL138_lpscTransition(PSC0, DOMAIN0, LPSC_DMAX, PSC_ENABLE);
	PRU_stop(PRU0);
	PRU_reset(PRU0);
	PRU_load(PRU0_PROG, PRUCode, sizeof(PRUCode)/sizeof(uint32_t));
	PRU_reset(PRU0);
	PRU_run(PRU0);

	// VPIF (95) interrupt serviced by INT4
	ICR = 0x010; // clear pending interrupts
	IER |= 0x010; // enable interrupt on line    

	// PRU (6) interrupt serviced by INT6
	ICR = 0x040; // clear pending interrupts
	IER |= 0x040; // enable interrupt on line    
	
	pic_data = (int *)ADDR_VIDEO_DATA_BASE;

    Image_data = (bgr *)(IMAGE_DATA_MEM);
    
	Thres_Image = (unsigned char *)(THRES_IMAGE_MEM); 
		
	Linux_Image = (bgr *)(ADDR_VIDEO_DATA_BASE+LINUX_IMAGE_OFFSET);
	
	LCD_Image = (bgr *)(ADDR_VIDEO_DATA_BASE+LCD_IMAGE_OFFSET);
	
	ptrshrdmem = (sharedmemstruct *)SHARED_MEM;	

    //**********************************************************************//
	// set specs for a blue golf ball
	specs_h = 166;
	specs_hrad = 45;
	if((specs_h-specs_hrad)<0) // wrap 0->360
	{
		specs_h2=specs_h+256;
	}
	else // wrap 360->0
	{
		specs_h2=specs_h-256;
	}
	specs_s = 69;
	specs_srad = 43;
	specs_v = 150;
	specs_vrad = 104;
    //**********************************************************************//

	while (CHKBIT(VPIF->INTSTAT,INT_FRAME_CH1) == 0) {}
	SETBIT(VPIF->INTSTATCLR, INT_FRAME_CH1);
	
}

int VPIF_beginning_line = 1;
void VPIF_HWI(void) {

	if (VPIF_beginning_line == 1) {
		// Invalidate Destination
		BCACHE_inv((void *)Image_data,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
		VPIF_beginning_line = 0;
	}
	PRU_wakeup(PRU0);
	// at times I had to run the below lines to wake up the PRU ????
//	PRU_stop(PRU0);
//	PRU_run(PRU0);
	SETBIT(VPIF->INTSTATCLR, INT_FRAME_CH1);
}

void PRU_HWI(void) {
	VPIF_beginning_line = 1;
	SWI_post(&SWI_vision);
	EVTCLR0 |= PRU_INTCLR;
	
}

// This function is called each time the DSP receives a new picture 
void userProcessColorImageFunc_laser(bgr *ptrImage) {
	
	int i;	

	if (ptrImage != NULL) {

		//**********************************************************************//
        if (colortoggle == 1) {
			colortoggle = 0;
		} else {
			colortoggle = 1;
		}
        
		if (colortoggle == 0) {
			// blue
			specs_h = 163;
			specs_hrad = 26;
			if((specs_h-specs_hrad)<0) // wrap 0->360
			{
				specs_h2=specs_h+256;
			}
			else // wrap 360->0
			{
				specs_h2=specs_h-256;
			}
			specs_s = 86;
			specs_srad = 43;
			specs_v = 158;
			specs_vrad = 97;
		}
		else {
			// orange
			specs_h = 7;
			specs_hrad = 7;
			if((specs_h-specs_hrad)<0) // wrap 0->360
			{
				specs_h2=specs_h+256;
			}
			else // wrap 360->0
			{
				specs_h2=specs_h-256;
			}
			specs_s = 236;
			specs_srad = 19;
			specs_v = 222;
			specs_vrad = 33;
		}
        //**********************************************************************//
        
		// Initialize all arrays for equivalency
		for (i=0; i < MAX_NUM_EQUIVALENCIES; i++) {
			equivalency_objects[i] = 0; // initialze link array
			object_stats[i].num_pixels_in_object = 0;
			object_stats[i].sum_r = 0;
			object_stats[i].sum_c = 0;
		}

		num_unique_objects = 0;
		
		object_detected = 0;
		
        // First Pass thru image.  Convert RGB to HSV.  This code is taking into account that the robot's camera only returns
        // a value between 16 and 240 for pixel intensity.  It also adds a gain of 2 to the blue intensity.  
		for (r=0;r<IMAGE_ROWS;r++) {
			for(c=0;c<IMAGE_COLUMNS;c++) {
				
				red =  ((ptrImage[r*IMAGE_COLUMNS+c].red - 16)*255)/224;
				green = ((ptrImage[r*IMAGE_COLUMNS+c].green - 16)*255)/224;
				blue = ptrImage[r*IMAGE_COLUMNS+c].blue*2;

				if (blue > 240) {
					blue = 240;
				}
				blue = ((blue - 16)*255)/224;

				min = my_min( red, green, blue );
	 			value = my_max( red, green, blue );

			 	delta = value - min;
			 	if( value != 0 ) {
			 		sat = (delta*255) / value;		// s

			 		if (delta != 0) {
					 	if( red == value )
					 		hue = 60*( green - blue ) / delta;		// between yellow & magenta
					 	else if( green == value )
					 		hue = 120 + 60*( blue - red ) / delta;	// between cyan & yellow
					 	else
					 		hue = 240 + 60*( red - green ) / delta;	// between magenta & cyan
					 	if( hue < 0 )
					 		hue += 360;
					} else {
					 	hue = 0;
					 	sat = 0;
					}
			 	} else {
			 		// r = g = b = 0		// s = 0, v is undefined
			 		sat = 0;
			 		hue = 0;
			 	}

			 	hue = (hue*255)/360;


				if ( (abs(sat-specs_s)<=specs_srad)
                  && (abs(value-specs_v)<=specs_vrad)
                  && (   (abs(hue-specs_h)<=specs_hrad)
                      || (abs(hue-specs_h2)<=specs_hrad) // catch the hue wraparround
                     )
                   ) { 			 	

					object_detected = 1;	// Set a flag that at least one pixel found above threshold
					
					// -------- Connectivity calculations ------------
					// Labels pixels 1 to MAX_NUM_EQUIVALENCIES depending on top and left neighbors
					// The labels represent object number...
					if (r == 0) top = 0;  else top = Thres_Image[(r-1)*IMAGE_COLUMNS+c];  // previous row, same column												
					if (c == 0) left = 0; else left = Thres_Image[r*IMAGE_COLUMNS+(c-1)];  // same row, previous column
					
					neighbor_type = 0;
					if (left != 0) neighbor_type += 1;
					if (top != 0)  neighbor_type += 2;
													
					current_object = 0;				
					switch (neighbor_type) {
						case 0: // Both neighbors zero, New object needed
                            if (num_unique_objects < (MAX_NUM_EQUIVALENCIES-1) ) {
				            	num_unique_objects++;
                                equivalency_objects[num_unique_objects] = num_unique_objects;
				            } else {
                                too_many_objects++;
                            }
							current_object = num_unique_objects;
							break;	
						case 1:	// Top is zero, left is not zero
							current_object = left;
							break;
						case 2:	// Left is zero, top is not zero
							current_object = top;
							break;
						case 3:	// Top and left are not zero... must note equivalency
							if (top == left) current_object = left;
							else {
								if (Check_Equivalency(top,left) == 0) {
									current_object = Set_Equivalency(top,left);																
								}
								else {
									current_object = left; 
								}
							}
							break;
						default: // Should NEVER enter here
							current_object = 0;  // Object 0 stores errors
							break;
					}
					Thres_Image[r*IMAGE_COLUMNS+c] = current_object;
					object_stats[current_object].num_pixels_in_object +=1;
					object_stats[current_object].sum_r += r;						
					object_stats[current_object].sum_c += c;						
					// ---------- Done with connectivity calculations (first pass) ----------
												
				} else {
					Thres_Image[r*IMAGE_COLUMNS+c] = 0;
				}
				
				
				
			 }
		}

		// initialize final object stats
		for (i=1; i<= MAX_NUM_OBJECTS; i++) {
			final_object_stats[i].sum_r = 0;
			final_object_stats[i].sum_c = 0;
			final_object_stats[i].num_pixels_in_object = 0;
			final_object_stats[i].center_r = 0.0;
			final_object_stats[i].center_c = 0.0;
			final_object_stats[i].C02_sum = 0.0;
			final_object_stats[i].C11_sum = 0.0;
			final_object_stats[i].C20_sum = 0.0;
			final_object_stats[i].theta = 0.0;
		}

		if (object_detected == 0) {
			num_unique_objects = 0;
		}
		else {
			num_unique_objects = Fix_Equivalency(num_unique_objects);// num_unique_objects contains the number of initial equivalencies found
		}
		
        if (num_unique_objects > MAX_NUM_OBJECTS) num_unique_objects = MAX_NUM_OBJECTS;


		// Third pass: correct image for nice display and calculate object moments
        // This is commented out because for the 176X144 image this adds a large amount of proccessing time when a large blob is found
//		for (r=0; r < IMAGE_ROWS; r++) {	// Loop over rows
//			for (c=0; c < IMAGE_COLUMNS; c++) {		// Loop over columns
//				if (Thres_Image[r*IMAGE_COLUMNS+c] > 0) {
//					// Fix pixel equivalency
//					Thres_Image[r*IMAGE_COLUMNS+c] = equivalency_objects[Thres_Image[r*IMAGE_COLUMNS+c]];
//
//					// Calculate second moments here
//					if ((Thres_Image[r*IMAGE_COLUMNS+c] > 0) && (Thres_Image[r*IMAGE_COLUMNS+c] <= MAX_NUM_OBJECTS)) {
//						final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].C02_sum += (c - final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].center_c)*(c - final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].center_c);
//						final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].C11_sum += (r - final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].center_r)*(c - final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].center_c);
//						final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].C20_sum += (r - final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].center_r)*(r - final_object_stats[Thres_Image[r*IMAGE_COLUMNS+c]].center_r);
//					}
//
//				}
//			}
//		}

		// Find largest object and calculate object orientation
		largest_num_pixels = 0;
		largest_object = 1;
		for (k = 1; k <= num_unique_objects ; k++) {				
			if (final_object_stats[k].num_pixels_in_object > largest_num_pixels) {
				largest_num_pixels = final_object_stats[k].num_pixels_in_object;
				largest_object = k;
			}	
			  // find theta of found blob
			  // commented out because moments need to be calculated above to use this code.
//            if (final_object_stats[k].num_pixels_in_object > NUMPIXELS_TO_CALC_ANGLE) {
//                // Calculate the object orientation angle if there are NUMPIXELS_TO_CALC_ANGLE in the object
//                Cdifference = final_object_stats[k].C20_sum - final_object_stats[k].C02_sum;
//                if (Cdifference != 0.0F) { // can't divide by zero
//                    final_object_stats[k].theta = atansp(final_object_stats[k].C11_sum/Cdifference)/2.0F;
//                } else {
//                    final_object_stats[k].theta = 0.0;
//                }
//                if (final_object_stats[k].C20_sum > final_object_stats[k].C02_sum) {
//                    if (final_object_stats[k].theta < 0) final_object_stats[k].theta += PI/2.0F;
//                    else final_object_stats[k].theta += -PI/2.0F;
//                }
//            } else {
//                final_object_stats[k].theta = 0;
//            }
            
		} // Ends loop through objects
		
		// Find the middle
		rbar = (int) (final_object_stats[largest_object].center_r);
		cbar = (int) (final_object_stats[largest_object].center_c);

        //**********************************************************************//
		// pass data to RobotControl()
		if (new_coordata == 0) {
			if (final_object_stats[largest_object].num_pixels_in_object > 1) {
				noimagefound = 0;
				new_num_found_objects = num_unique_objects;
				if (colortoggle == 0) {
//					bobject_x = cbar - IMAGE_COLUMNS/2;
					bobject_x = cbar - IMAGE_COLUMNS*0.6;
					bobject_y = rbar - IMAGE_ROWS/2;
					bnumpels = final_object_stats[largest_object].num_pixels_in_object;
				}
				else if (colortoggle == 1) {
//					oobject_x = cbar - IMAGE_COLUMNS/2;
					oobject_x = cbar - IMAGE_COLUMNS*0.4;
					oobject_y = rbar - IMAGE_ROWS/2;
					onumpels = final_object_stats[largest_object].num_pixels_in_object;
				}
                
				//new_object_theta 	= final_object_stats[largest_object].theta;
				//new_C20				= final_object_stats[largest_object].C20_sum;
				//new_C02				= final_object_stats[largest_object].C02_sum;
				new_coordata = 1;
			} else {
				noimagefound = 1;
				new_num_found_objects = num_unique_objects;
				if (colortoggle == 0) {
					bobject_x = 0.0;
					bobject_y = 0.0;
					bnumpels = 0;
				}
				else if (colortoggle == 1) {
					oobject_x = 0.0;
					oobject_y = 0.0;
					onumpels = 0;
				}
				//new_object_theta = 0.0;
				//new_C20 = 0.0;
				//new_C02 = 0.0;
				new_coordata = 1;
			}
		}
        //**********************************************************************//

        //create red x for largest centroid position
        if (final_object_stats[largest_object].num_pixels_in_object > 6) {
            
            ptrImage[rbar*IMAGE_COLUMNS+cbar].red = 0;
            ptrImage[rbar*IMAGE_COLUMNS+cbar].blue = 0;
            ptrImage[rbar*IMAGE_COLUMNS+cbar].green = 255;
            if (rbar > 0) {
                ptrImage[(rbar-1)*IMAGE_COLUMNS+cbar].red =  0;
                ptrImage[(rbar-1)*IMAGE_COLUMNS+cbar].blue = 0;
                ptrImage[(rbar-1)*IMAGE_COLUMNS+cbar].green = 255;
            }
            if (rbar < (IMAGE_ROWS-1)) {
                ptrImage[(rbar+1)*IMAGE_COLUMNS+cbar].red = 0;
                ptrImage[(rbar+1)*IMAGE_COLUMNS+cbar].blue = 0;
                ptrImage[(rbar+1)*IMAGE_COLUMNS+cbar].green = 255;
            }
            if (cbar > 0) {
                ptrImage[rbar*IMAGE_COLUMNS+(cbar-1)].red = 0;
                ptrImage[rbar*IMAGE_COLUMNS+(cbar-1)].blue = 0;
                ptrImage[rbar*IMAGE_COLUMNS+(cbar-1)].green = 255;
            }
            if (cbar < (IMAGE_COLUMNS-1)) {
                ptrImage[rbar*IMAGE_COLUMNS+(cbar+1)].red = 0;
                ptrImage[rbar*IMAGE_COLUMNS+(cbar+1)].blue = 0;
                ptrImage[rbar*IMAGE_COLUMNS+(cbar+1)].green = 255;
            }
        }


        if ( 12 == switchstate) {
            UpdateLCDwithLADAR(ptrImage,1);
        }

        // Send image to Color LCD if LCD ready for new data
		if (updateLCD) {

			updateLCD = 0;

			BCACHE_inv((void *)(ADDR_VIDEO_DATA_BASE+0x1A900),IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
			// Flush or write back source
	        BCACHE_wb ((void *)ptrImage,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
			//Need to clean the cache here
			EDMA3_1_Regs->PARAMENTRY[33].OPT = 0x0011E00C;
			EDMA3_1_Regs->PARAMENTRY[33].SRC = (unsigned int)Image_data;
			EDMA3_1_Regs->PARAMENTRY[33].A_B_CNT = 0x004004A4;   //Maybe to ACNT = 1188 and BCNT = 64
			EDMA3_1_Regs->PARAMENTRY[33].DST = (ADDR_VIDEO_DATA_BASE+LCD_IMAGE_OFFSET);
			EDMA3_1_Regs->PARAMENTRY[33].SRC_DST_BIDX = 0x04A404A4;
			EDMA3_1_Regs->PARAMENTRY[33].LINK_BCNTRLD = 0x0000FFFF;  // Null link
			EDMA3_1_Regs->PARAMENTRY[33].SRC_DST_CIDX = 0x0;
			EDMA3_1_Regs->PARAMENTRY[33].CCNT = 0x1;  //Last command triggers transmission

		}

		// If Linux is ready for another full 176X144 RGB image start the EDMA transfer of the image to external memory
		if (GET_IMAGE_TO_LINUX) {

			// Invalidate Destination
			BCACHE_inv((void *)Linux_Image,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
			// Flush or write back source
	        BCACHE_wb ((void *)ptrImage,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);

			EDMA3_1_Regs->PARAMENTRY[32].OPT = 0x0011F00C;
			EDMA3_1_Regs->PARAMENTRY[32].SRC = (unsigned int)Image_data;
			EDMA3_1_Regs->PARAMENTRY[32].A_B_CNT = 0x004004A4;   //Maybe to ACNT = 1188 and BCNT = 64
			EDMA3_1_Regs->PARAMENTRY[32].DST = (ADDR_VIDEO_DATA_BASE+LINUX_IMAGE_OFFSET);
			EDMA3_1_Regs->PARAMENTRY[32].SRC_DST_BIDX = 0x04A404A4;
			EDMA3_1_Regs->PARAMENTRY[32].LINK_BCNTRLD = 0x0000FFFF;  // Null link
			EDMA3_1_Regs->PARAMENTRY[32].SRC_DST_CIDX = 0x0;
			EDMA3_1_Regs->PARAMENTRY[32].CCNT = 0x1;  //Last command triggers transmission

		}
		
	}  // Ends if statement to see if image pointer is null				
	
}

void vision(void) {
    
	//SETLED3;
	
	// 144X176 BGR image ready for processing
	userProcessColorImageFunc_laser(Image_data);
	
//	CLRLED3;
}

// No reason for these to be global except that it is easier to debug/view these variables in CCS when global
int current;
int equivalency = 0;
int done = 0;

int Check_Equivalency(int A, int B) {	// Looks through the link array starting at A 
										// to see if A and B are equivalent					
	done = 0;
	equivalency = 0;
	current = equivalency_objects[A];
	while (done == 0) {
		if (current == A) {
			done = 1;
		}	
		else {
			if (current == B) {
				equivalency = 1;
				done = 1;
			}
			else current = equivalency_objects[current];
		}	
	}
	return (equivalency);			
}

int Set_Equivalency (int A, int B) {  	
	// Modifies link list so that A and B are equivalent
	// and returns higher of A and B
 	// Operations:
 	//		1:  temp set to value at A
 	//		2:  A set to value at B
 	//		3:  B set to temp
	// NOTE: Does NOT work if equivalence between A and B already done!
	int temp = equivalency_objects[A];
	
	equivalency_objects[A] = equivalency_objects[B];
	equivalency_objects[B] = temp;
	if (A > B) return(A);
	else return(B);
}



int Fix_Equivalency(int num_equivalencies_used) { // Fixes equivalency to ordered values, returns num. objects..

	int i;
    int ordered_num = 1;
    int num_unique = 0;
    int done = 0;
    int current = 0;
    
    int num_equivs = num_equivalencies_used+1;

    // just in case invalid parameter sent
    if (num_equivs > MAX_NUM_EQUIVALENCIES) num_equivs = MAX_NUM_EQUIVALENCIES;
    if (num_equivs < 1) num_equivs = 1;
    
    // zero temp link array
    for (i=1; i < num_equivs; i++) {
        temp_equivalency_objects[i] = 0;
    }

    for (i=1; i < num_equivs; i++) {
        
        if ( (temp_equivalency_objects[i] == 0) && (equivalency_objects[i] != 0) ) {
            temp_equivalency_objects[i] = ordered_num;
            ordered_num++;
            done = 0;
            current = equivalency_objects[i];
            while (done == 0) {
                if (current == i) done = 1;
                else {
                    if (temp_equivalency_objects[current] == 0) {
                        temp_equivalency_objects[current] = temp_equivalency_objects[i];
                    }
                    current = equivalency_objects[current];
                }	
            }
        }
        
	}
    
    num_unique = ordered_num - 1;
	
	// Copy equivalencies over		
	for (i=1; i < num_equivs; i++) {
		equivalency_objects[i] = temp_equivalency_objects[i];
	}
    
    if ( num_unique > MAX_NUM_OBJECTS )  {
        num_unique = MAX_NUM_OBJECTS;
    }
	
	// Add up totals, since many different objects now refer to same final object 
	for (i=1; i < num_equivs; i++) {	
		if (equivalency_objects[i] <= num_unique) {
			final_object_stats[equivalency_objects[i]].num_pixels_in_object += object_stats[i].num_pixels_in_object;
			final_object_stats[equivalency_objects[i]].sum_r += object_stats[i].sum_r;
			final_object_stats[equivalency_objects[i]].sum_c += object_stats[i].sum_c;
		}
	}

	// Calculate the average pixel values
	for (i=1; i <= num_unique; i++) {
		if (final_object_stats[i].num_pixels_in_object !=0) {
			final_object_stats[i].center_r = final_object_stats[i].sum_r/final_object_stats[i].num_pixels_in_object;
			final_object_stats[i].center_c = final_object_stats[i].sum_c/final_object_stats[i].num_pixels_in_object;
		}
	}
	
	// Now have ordered_num-1 unique objects
	return(num_unique);
}


