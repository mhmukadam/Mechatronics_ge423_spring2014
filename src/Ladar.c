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
#include "evmomapl138_uart.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"

#include "mcbsp_com.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"
#include "LCDprintf.h"
#include "MatrixMath.h"
#include "xy.h"
#include "ladar.h"

//SWAP IS USED FOR LINE ALGORITHM -- this is a very fast & efficient routine to switch two numbers using XOR
#define SWAP(x,y) ((x) ^= (y) ^= (x) ^= (y))

// These are the corners used for drawing the map on the color LCD
#define cornersNum 10
float cornersX[cornersNum][2] = {{-5,-2},{ 2, 5},{-2,-6},{-6,-6},{-6, 6},{ 6, 6},{ 6, 2},{-2,-2},{-2, 2},{ 2, 2}};
float cornersY[cornersNum][2] = {{-3, 0},{ 0,-3},{ 0, 0},{ 0,12},{12,12},{12, 0},{ 0, 0},{ 8, 4},{ 4, 4},{ 4, 8}};

// These are the corners used for corner finding
float corner_x[6] = {6, 6,-6,-6,2,-2};
float corner_y[6] = {0,12,12, 0,4, 4};

float scalingLadar = 4; //px per foot -- this can be updated to zoom in or out.

//int numberOfBlocks = 200;

volatile int LADARready = 1;
volatile int LADARprocess = 0;

volatile int uart1count = 0;

char LADARinBuffer[LADAR_IN_BUFF_SIZE];
char LADARoutBuffer[LADAR_OUT_BUFFER_SIZE];		//Commands to be sent to LADAR
int numReadings = 0;
float errL=0,errC=0,errR=0;
float tLADARleftFt=0,tLADARcenterFt=0,tLADARrightFt=0;
char cPrevLADARdata=0;
char cLADARdata=0;
int iLADARrcv=0;
char cCommandName=0;
float LADARleftFt = -.99, LADARcenterFt = -.99, LADARrightFt = -.99;
const float fLEFT_RIGHT_ANGLE = 60.0;  // this variable sets the degrees from center to the LADARleftFt measurement and LADARrightFt
float newLADARdistance[LADAR_MAX_DATA_SIZE];  //in mm
float newLADARangle[LADAR_MAX_DATA_SIZE];		// in degrees
pose ROBOTps;
pose LADARps;
int numreadings_lastscan = 0;
float startAngle=0, stepSize=0;
float newLADARdataX[LADAR_MAX_DATA_SIZE];
float newLADARdataY[LADAR_MAX_DATA_SIZE];
int newLADARdata = 0;
float robotX=0, robotY=0, robotT=0;

extern EDMA3_CCRL_Regs *EDMA3_0_Regs;


int init_LADAR(void) {
	int i;
	int rtn;
    int index;
    uint8_t dummy = 0;

	
	for (i = 0;i<LADAR_IN_BUFF_SIZE;i++){
		LADARinBuffer[i] = 0xff;
	}
	
    rtn = UART_init(UART1, 19200);
	if (rtn != ERR_NO_ERROR)
	{
	  return (rtn);
	}

	UART_txString(UART1, "S1152000000000\n ");

	index = 0;
	while ((uart1count < 18) && (index < 1000000)) {
		if (UART_rxByte(UART1,&dummy)==0){
			uart1count++;
		}
		index++;
	}

    rtn = UART_init(UART1, 115200);
	if (rtn != ERR_NO_ERROR)
	{
	  return (rtn);
	}

		// enable DMA region access UART1
	EDMA3_0_Regs->DRA[1].DRAE |= (1U<<EDMA3EVT_UART1_RECEIVE);
	// clear pending events
	EDMA3_0_Regs->SHADOW[1].ECR = (1U<<EDMA3EVT_UART1_RECEIVE);
	EDMA3_0_Regs->SHADOW[1].SECR = (1U<<EDMA3EVT_UART1_RECEIVE);
	// enable event interrupt
	EDMA3_0_Regs->SHADOW[1].IESR = (1U<<EDMA3EVT_UART1_RECEIVE);
	// enable event
	EDMA3_0_Regs->SHADOW[1].EESR = (1U<<EDMA3EVT_UART1_RECEIVE);
	// clear interrupts
	EDMA3_0_Regs->SHADOW[1].ICR = (1U<<EDMA3EVT_UART1_RECEIVE);
	
	LADARps.x = 3.5/12; // 3.5/12 for front mounting
	LADARps.y = 0;
	LADARps.theta = 1;  // not inverted

	for(i = 0;i<LADAR_MAX_DATA_SIZE;i++)
	{ newLADARdistance[i] = LADAR_MAX_READING; } //initialize all readings to max value.
	
	return (0);
}

// function to return sign of a float
float signf(float n) {
	return (n>=0)?1:-1;
}


void LADARtask(void) {
	TSK_sleep(100);
	while(1) {
		if(LADARready) {
			LADARready = 0;
			LADAR_scan(-120.0, 120.0);
		} else if(LADARprocess) {
			// process
			LADAR_Get_and_Process_Scan();
			newLADARdata = 1;
			LADARprocess = 0;
			LADARready = 1;
		}
		TSK_sleep(1);
	}
}

/*
 *  FUNCTION: int LADAR_scan(float startSweepDeg, float endSweepDeg, REMOVED-int numDistBlocks-)
 *
 *  PARAMETERS: angles where to start sweeping and where to end. Also
 *              takes the number of blocks it should divide the data into (now defined as NUMDISTBLOCKS).
 *              Angles are given in degrees and range from [-119.5 , 119.5] and the number of
 *				blocks ranges from []. NOTE: the zero degree line
 *				is pointing towards the front of the sensor.
 *
 *  DESCRIPTION: the sweep goes from startSweepDeg angle to the endSweepDeg angle. It
 *               divides the read distances into blocks, returning the shortest
 *				distance reading per block.
 *
 *  RETURNS: 1 if the ending point is smaller than the starting point or the
 *			 the arguments are out of bound.
 *
 */
int LADAR_scan(float startSweepDeg, float endSweepDeg)
{
	// Local variables
	int startCount_ 	= 0;
	int endCount_ 		= 0;
	int numClustered_ 	= 0;

	// Check if the parameters are within bounds
	if( -120 		 <=startSweepDeg&&
		startSweepDeg<= endSweepDeg	&&
		endSweepDeg	 <= 120			&&
  	    1 			 <= NUMDISTBLOCKS  )
	{
		// Convert the parameters
		startCount_ = floorf((startSweepDeg+135.0)*768.0/270.0);
		endCount_ = ceilf((endSweepDeg+135.0)*768.0/270.0);
		numClustered_ = floorf((endCount_-startCount_)/(float)(NUMDISTBLOCKS));
		if(numClustered_ == 0)
		{ numClustered_ = 1;}
		// Send the scan command to the LADAR
		return LADAR_sendG(startCount_,endCount_,numClustered_);
	}

	return 1;
}

/*
 *  FUNCTION: int LADAR_sendG(int startPoint, int endPoint, int clusterCount)
 *
 *  PARAMETERS: sweep starting point between [0-768], sweep ending point
 *              between [0-768] and the number of neighbors to cluster into a reading.
 *				the smallest distance reading in a cluster is returned.
 *
 *  DESCRIPTION: the LADAR sweeps from point "startPoint" to "endPoint". The
 *				 "clusterCount" will command the LADAR to return the closest reading
 *				 every "clusterCount" measurements.
 *
 *  RETURNS: 1 if the ending point is smaller than the starting point.
 *
 */
int LADAR_sendG(int startPoint, int endPoint, int clusterCount)
{

	int numPoints = 0;
	int numSets = 0;
	int remPoints = 0;
	int numBytes = 0;
	// Ensure parameters stay within possible limits
	if(startPoint < 44){startPoint = 44;}
	if(endPoint > 725){endPoint = 725;}
	if(clusterCount > 99){clusterCount = 99;}
	if(clusterCount < 1){clusterCount = 1;}

	// If the difference between endPoint-startPoint is not positive we can't sweep
	if( (endPoint-startPoint) < 0 ){ return 1;}


	numPoints = (endPoint-startPoint)/clusterCount;
	numPoints++;
	numSets = numPoints/32;
	numBytes = 12 + numSets*65;
	remPoints = (numPoints%32);
	if (remPoints == 0) {
		numBytes++;
	} else {
		numBytes = numBytes + remPoints*2 + 2;
	}

	// set up edma for ladar
	BCACHE_inv(LADARinBuffer, sizeof(LADARinBuffer), 1);
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].OPT = OPT_TCINTEN | (EDMA3EVT_UART1_RECEIVE<<OPT_TCC_SHIFT);// | OPT_SYNCDIM;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].SRC = 0x01D0C000;//(uint32_t)UART1->RBR;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].A_B_CNT = 0x1 | (numBytes<<16);//0x00120001; // A = 1 bytes, B = 16
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].DST = (uint32_t)&LADARinBuffer[0];
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].SRC_DST_BIDX = 0x00010000;  // Offset 1 byte at dest.
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].LINK_BCNTRLD = 0x0000FFFF;  // Null Link
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].SRC_DST_CIDX = 0x0;
	EDMA3_0_Regs->PARAMENTRY[EDMA3EVT_UART1_RECEIVE].CCNT = 0x1;

	if(startPoint < 100) {
		sprintf(LADARoutBuffer,"G0%2.0d",startPoint);
	} else {
		sprintf(LADARoutBuffer,"G%3.0d",startPoint);
	}

	if (endPoint < 100) {
		sprintf(LADARoutBuffer,"%s0%2.0d",LADARoutBuffer,endPoint);
	} else {
		sprintf(LADARoutBuffer,"%s%3.0d",LADARoutBuffer,endPoint);
	}

	if (clusterCount < 10) {
		sprintf(LADARoutBuffer,"%s0%1.0d\n",LADARoutBuffer,clusterCount);
	} else {
		sprintf(LADARoutBuffer,"%s%2.0d\n",LADARoutBuffer,clusterCount);
	}

	UART_txString(UART1,LADARoutBuffer);

	return 0;
}

/*
 *  FUNCTION: void LADARReceiveCharTask(void)
 *
 *  PARAMETERS: void
 *
 *  DESCRIPTION: task-called function that reads and parses the incoming
 *               buffer from the LADAR
 *
 *  RETURNS: void
 *
 */
void LADAR_Get_and_Process_Scan(void)
{
	int i;
	char tempStr[4] = "999";
	int startingstep;
//	int endingstep;
	int clustercount;
	// initialize buffer variables
	errL = errC = errR = 900.0;
	tLADARleftFt = tLADARcenterFt = tLADARrightFt = -99.0;
	iLADARrcv = 0;
	cPrevLADARdata = cLADARdata = '\0';

	while (LADARinBuffer[iLADARrcv] != '\n' || LADARinBuffer[iLADARrcv+1] != '\n') {
		iLADARrcv++;
		if (iLADARrcv > 1399) {
			return;
		}
	}
	iLADARrcv++;

	// find beginning of command
	cCommandName = '\0';
	i = 0;
	while('\0' == cCommandName)
	{
			cLADARdata = LADARinBuffer[i];
			if( cLADARdata == 'G' )
			{ cCommandName = cLADARdata; }
			i++;
			if(i > iLADARrcv)//error case, never received 'G' character
			{ return; }
	}
	// parse out the echo
	strncpy( tempStr, &LADARinBuffer[i], 3);
	startingstep = atoi(tempStr);
//	strncpy( tempStr, &LADARinBuffer[i+3], 3);
//	endingstep = atoi(tempStr);
	strncpy( tempStr, &LADARinBuffer[i+6], 2);
	tempStr[2] = '\0';
	clustercount = atoi(tempStr);

	// calculate constants
	stepSize = 270.0/768.0*clustercount;
	startAngle = startingstep *270.0/768.0-135.0 + stepSize/2;

	i = i+8;  //skip next two data sections (each terminated by a '\n' character)
	while('\n' != LADARinBuffer[i++])
	{
			if(i > iLADARrcv)//error case: no '\n'
			{ return; }
	}
	while('\n' != LADARinBuffer[i++])
	{
			if(i > iLADARrcv)//error case: no '\n'
			{ return; }
	}

	// parse out data 1 section at a time
	numReadings = 0;
	while(i+1 <= iLADARrcv)
	{
		// -- If first byte is '\n', this is end of one data section
		//    & if next is also a '\n', this is the end of all the data
		if('\n' == LADARinBuffer[i])
		{
			i++;	//move to the next byte
			if('\n' == LADARinBuffer[i])
			{
				//finished
				break;
			}
		}
		else
		{// -- --  decode two bytes  to floats
			 newLADARdistance[numReadings] = (float)decode2byteLADAR(&LADARinBuffer[i],(int)newLADARdistance[numReadings]);
			 newLADARangle[numReadings] = (startAngle + numReadings*stepSize)*LADARps.theta;

			 //find the left, center, and right values.
			 if( fabsf(newLADARangle[numReadings] - fLEFT_RIGHT_ANGLE) < errL )
			 {
				tLADARleftFt = newLADARdistance[numReadings]*LADAR_FEET_PER_MM;
				errL = fabsf(newLADARangle[numReadings] - fLEFT_RIGHT_ANGLE);
			 }
			 if( fabsf(newLADARangle[numReadings]) < errC )
			 {
				tLADARcenterFt = newLADARdistance[numReadings]*LADAR_FEET_PER_MM;
				errC = fabsf(newLADARangle[numReadings]);
			 }
			 if( fabsf(newLADARangle[numReadings] + fLEFT_RIGHT_ANGLE) < errR )
			 {
				tLADARrightFt = newLADARdistance[numReadings]*LADAR_FEET_PER_MM;
				errR = fabsf(newLADARangle[numReadings] + fLEFT_RIGHT_ANGLE);
			 }
			 numReadings++;
			 i+=2; // move to the next two bytes.
		}
	}

	if(numReadings >= 3)
	{	// upate the left, right center values
		LADARleftFt = tLADARleftFt;
		LADARcenterFt = tLADARcenterFt;
		LADARrightFt = tLADARrightFt;

		numreadings_lastscan = numReadings;

		// determine the XY positions of the reflections
		LADARdetections2xy (newLADARdataX, newLADARdataY, &ROBOTps, &LADARps,
						 newLADARangle, newLADARdistance, numReadings);
	}


	robotX = ROBOTps.x;   // MATLAB accessible variables.
	robotY = ROBOTps.y;
	robotT = ROBOTps.theta;

	return;
}

/*
 *  FUNCTION: decode2byteLADAR(char* ptr, int previousValue);
 *
 *  PARAMETERS: ptr to a high significant byte and a low significant byte chars
 *				and the previous value of the scan
 *
 *  DESCRIPTION: This function will merge the raw data comming from the LADAR
 *				 and create a distance value in [mm]
 *
 *  RETURNS: a distance value in [mm]
 *
 */
int decode2byteLADAR(char* ptr, int previousValue)
{
	/*
		Error Code Error Type
		0 Possibility of detected object is at 22m (ATB: is this a documentation mistake?)
		1 Reflected light has low intensity
		2 Reflected light has low intensity
		3 Reflected light has low intensity
		4 Reflected light has low intensity
		5 Reflected light has low intensity
		6 Possibility of detected object is at 5.7m
		7 Distance data on the preceding and succeeding steps have errors
		8 Others
		9 The same step had error in the last two scan
		10 Others
		11 Others
		12 Others
		13 Others
		14 Others
		15 Others
		16 Possibility of detected object is in the range 4096mm
		17 Others
		18 Unspecified
		19 Non-Measurable Distance
	*/

	// Variables
	int resultInt;
	char this_char, next_char;
	this_char = *ptr;
	next_char = *(ptr+1);

	resultInt = ((this_char - 0x30) << 6) + next_char - 0x30;

	if (resultInt < 20) {
		//flag_data[buff_pointer][dist_data_pointer] = datum;
		switch (resultInt) {
		case 0:			//Note we will never have a 22m distance
			resultInt = LADAR_MAX_READING; //= 22000;
			break;
		case 6:
			resultInt = LADAR_MAX_READING;
			break;
		case 16:
			resultInt = 4096;
			break;
		default:
			resultInt = previousValue;
		// leave unchanged (from previous scans)
		}
	}
	return resultInt;
}

/*
 *  FUNCTION: void LADARdetections2xy(float* X, float*Y, const pose *ROBOTps, const pose *LADARps,
 *							 const float* Langles, const float* Ldistances, const int iReadings);
 *
 *  PARAMETERS: float* X and float*Y are the reflection point arrays to be updated
 *				ROBOTps is the robot's position and heading
 *				LADARps is the position and orientation relative to the robot
 *				(theta of +1 represents the LADAR mounted upright, -1 upside down
 * 				Langles and Ldistances are arrays of (iReadings) LADAR measurments
 *
 *  DESCRIPTION: Calculates the XY locations in feet of points detected by the LADAR, given
 *	the position of the robot
 *
 *  RETURNS: updates the (iReadings) entries of X and Y in feet
 *
 */
void LADARdetections2xy (float* X, float*Y, const pose *ROBOTps, const pose *LADARps,
						 const float* Langles, const float* Ldistances, const int iReadings)
{
	int i;
	float xoffset, yoffset;
	xoffset = ROBOTps->x + (LADARps->x*cosf(ROBOTps->theta)-LADARps->y*sinf(ROBOTps->theta - PI/2.0));
	yoffset = ROBOTps->y + (LADARps->x*sinf(ROBOTps->theta)-LADARps->y*cosf(ROBOTps->theta - PI/2.0));

	for(i = 0; i<iReadings; i++)
	{
		X[i] = xoffset + Ldistances[i]*LADAR_FEET_PER_MM*cosf(Langles[i]*PI/180.0 + ROBOTps->theta);
		Y[i] = yoffset + Ldistances[i]*LADAR_FEET_PER_MM*sinf(Langles[i]*PI/180.0 + ROBOTps->theta);
	}

	return;
}

/*
 *  FUNCTION: void UpdateLCDwithLADAR(bgr *ptrImage,int use_world_ref_frame)
 *
 *  PARAMETERS: takes a ptr to a bgr image file that must be IMAGE_ROWS by IMAGE_COLUMNS large
 *				use_world_ref_frame = 1 means use World reference frame
 *				use_world_ref_frame = 0 means use Robots reference frame
 *
 *  DESCRIPTION: overwrites image at ptrImage with a black and green grid with lines
 *      representing two foor spacing.
 *		The robot is represented by a blue "+" and it's orientation with a white dot.  If
 *      more than three LADAR readings are present, the LADAR readings are displayed
 *      as red dots (with increasing magnitude if more than one beam hit this pixel)
 *	   If the reading is offscreen, the LADAR point is truncated to the
 *	   edge of the screen and drawn in blue.  Finally, the LEFT, CENTER,
 *	   and RIGHT measurements are drawn in yellow
 *
 *  RETURNS: void.  ptrImage is updated and can be sent on to the LCD screen if you want to.
 *
 */
int tempSign = -1;
float tempOff = PI/2;
void UpdateLCDwithLADAR(bgr *ptrImage, int use_world_ref_frame)
{
	int c, r;
	float tempFy, tempFx;
	int tempRed;
	int roboXs[4],roboYs[4];
	int offsetX = IMAGE_COLUMNS/2.0;
	int offsetY = IMAGE_ROWS*1/3;
	int roboX, roboY,i;
	//int roundSF = 0; // scaling factor for grid lines
	float xoffset = 0, yoffset = 0;
	pose ROBOT_LCDps;

	if (use_world_ref_frame == 1) {
		ROBOT_LCDps.x = ROBOTps.x;
		ROBOT_LCDps.y = ROBOTps.y;
		ROBOT_LCDps.theta = ROBOTps.theta;

		xoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
		yoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));

	} else {
		ROBOT_LCDps.x = 0;
		ROBOT_LCDps.y = 0;
		ROBOT_LCDps.theta = PI/2.0;
	}
	//print a grid with 2 foot spacing to the screen
	//roundSF = (unsigned int)(2*(scalingLadar)+0.5);
	for (r=0;r<IMAGE_ROWS;r++) {
		for(c=0;c<IMAGE_COLUMNS;c++) {
			if(((r%8) == 0) || ((c%8) == 0))  // WAS if(((r%roundSF) == 0) || ((c%roundSF) == 0)) // this seemed to cause optimization issues
			{
				ptrImage[r*IMAGE_COLUMNS+c].red = 0;
				ptrImage[r*IMAGE_COLUMNS+c].blue = 0;
				ptrImage[r*IMAGE_COLUMNS+c].green = 60;
			}
			else
			{
				ptrImage[r*IMAGE_COLUMNS+c].red = 0;
				ptrImage[r*IMAGE_COLUMNS+c].blue = 0;
				ptrImage[r*IMAGE_COLUMNS+c].green = 0;
			}
		}
	}


	// print robot location and orientation
	if(numreadings_lastscan > 0)
	{
		roboX = offsetX - ROBOT_LCDps.x*scalingLadar;  //center of robot is (roboX,roboY)
		roboY = offsetY + ROBOT_LCDps.y*scalingLadar;
		roboXs[0] = roboX-1;	//draw crosshair around robot.
		roboXs[1] = roboX;
		roboXs[2] = roboX+1;
		roboXs[3] = roboX;
		roboYs[0] = roboY;
		roboYs[1] = roboY-1;
		roboYs[2] = roboY;
		roboYs[3] = roboY+1;
		for(i = 0; i<4; i++) //paint robot blue
		{
			LCD_saturate(&roboYs[i],&roboXs[i]);
			ptrImage[roboYs[i]*IMAGE_COLUMNS+roboXs[i]].red = 60;
			ptrImage[roboYs[i]*IMAGE_COLUMNS+roboXs[i]].blue = 255;
			ptrImage[roboYs[i]*IMAGE_COLUMNS+roboXs[i]].green = 60;
		}
		//paint robot orientation marker
		roboX = roboX-floorf(2*cosf(ROBOT_LCDps.theta)+0.5);
		roboY = roboY+floorf(2*sinf(ROBOT_LCDps.theta)+0.5);

		LCD_saturate(&roboY,&roboX);
		ptrImage[roboY*IMAGE_COLUMNS+roboX].red = 255;
		ptrImage[roboY*IMAGE_COLUMNS+roboX].blue = 255;
		ptrImage[roboY*IMAGE_COLUMNS+roboX].green = 255;
	}

	// print LADAR readings
	for( i=0;i<numreadings_lastscan;i++){

			c = offsetX - roundFloatToInt(scalingLadar*(xoffset + newLADARdistance[i]*LADAR_FEET_PER_MM*cosf(newLADARangle[i]*PI/180.0 + ROBOT_LCDps.theta)));
			r = offsetY + roundFloatToInt(scalingLadar*(yoffset + newLADARdistance[i]*LADAR_FEET_PER_MM*sinf(newLADARangle[i]*PI/180.0 + ROBOT_LCDps.theta)));

			if (LCD_saturate(&r,&c) == 0) {
				tempRed = ptrImage[r*IMAGE_COLUMNS+c].red;
				if( tempRed < 160)
					ptrImage[r*IMAGE_COLUMNS+c].red = 160;
				else
					ptrImage[r*IMAGE_COLUMNS+c].red = (tempRed + 20 > 255) ? 255 : tempRed + 20;
			}
			else
			{
				ptrImage[r*IMAGE_COLUMNS+c].blue = 255;
			}
	}

	// draw course walls on screen
	if(0== use_world_ref_frame)
	{
		for( i=0;i<cornersNum;i++)
		{
			//rotate and tranlate the lines.	 ROBOTps
			float thetaN = tempOff+tempSign*ROBOTps.theta;
			float xs = (-ROBOTps.x + cornersX[i][0])*cosf(thetaN)-(-ROBOTps.y + cornersY[i][0])*sinf(thetaN);
			float ys = (-ROBOTps.x + cornersX[i][0])*sinf(thetaN)+(-ROBOTps.y +cornersY[i][0])*cosf(thetaN);

			float xe = (-ROBOTps.x + cornersX[i][1])*cosf(thetaN)-(-ROBOTps.y +cornersY[i][1])*sinf(thetaN);
			float ye = (-ROBOTps.x + cornersX[i][1])*sinf(thetaN)+(-ROBOTps.y +cornersY[i][1])*cosf(thetaN);
			//scale to LCD screen
			int x0 =  offsetX - roundFloatToInt(xs*scalingLadar);
			int y0 =  offsetY + roundFloatToInt(ys*scalingLadar);
			int x1 =  offsetX - roundFloatToInt(xe*scalingLadar);
			int y1 =  offsetY + roundFloatToInt(ye*scalingLadar);

			Bresenhamline(x0,y0,x1,y1, ptrImage);
		}

	}

	// highlight LEFT, CENTER, and RIGHT measurements
	tempFx = offsetX - scalingLadar*(ROBOT_LCDps.x + (LADARps.x*cosf(ROBOT_LCDps.theta)-LADARps.y*sinf(ROBOT_LCDps.theta - PI/2.0)));
	tempFy = offsetY + scalingLadar*(ROBOT_LCDps.y + (LADARps.x*sinf(ROBOT_LCDps.theta)-LADARps.y*cosf(ROBOT_LCDps.theta - PI/2.0)));

	roboXs[0] = roundFloatToInt(tempFx - LADARleftFt  *scalingLadar*cos(ROBOT_LCDps.theta+fLEFT_RIGHT_ANGLE*PI/180.0));
	roboXs[1] = roundFloatToInt(tempFx - LADARcenterFt*scalingLadar*cos(ROBOT_LCDps.theta));
	roboXs[2] = roundFloatToInt(tempFx - LADARrightFt *scalingLadar*cos(ROBOT_LCDps.theta-fLEFT_RIGHT_ANGLE*PI/180.0));

	roboYs[0] = roundFloatToInt(tempFy + LADARleftFt  *scalingLadar*sin(ROBOT_LCDps.theta+fLEFT_RIGHT_ANGLE*PI/180.0));
	roboYs[1] = roundFloatToInt(tempFy + LADARcenterFt*scalingLadar*sin(ROBOT_LCDps.theta));
	roboYs[2] = roundFloatToInt(tempFy + LADARrightFt *scalingLadar*sin(ROBOT_LCDps.theta-fLEFT_RIGHT_ANGLE*PI/180.0));

	for(i = 0; i<3; i++)
		{
			//saturate the values
			LCD_saturate(&roboYs[i],&roboXs[i]);

			ptrImage[roboYs[i]*IMAGE_COLUMNS+roboXs[i]].red = 255;
			ptrImage[roboYs[i]*IMAGE_COLUMNS+roboXs[i]].blue = 0;
			ptrImage[roboYs[i]*IMAGE_COLUMNS+roboXs[i]].green = 255;
		}
	return;
}

/*
 *  FUNCTION: void LCD_saturate(int *r,int *c)
 *
 *  PARAMETERS: r, c, LCD pixel values
 *
 *  DESCRIPTION: Saturate r and c pixel values to be on screen
 *
 *  RETURNS: 1 if saturated, 0 if unchanged
 *
 */
#define FRM_CNSTC   13
#define FRM_CNSTR   5
int LCD_saturate(int *r,int *c) {
	int tmpr=*r,tmpc=*c;


	*c = (*c < FRM_CNSTC )? FRM_CNSTC : *c;
	*c = (*c > IMAGE_COLUMNS-FRM_CNSTC )? IMAGE_COLUMNS-FRM_CNSTC : *c;
	*r = (*r < FRM_CNSTR )? FRM_CNSTR : *r;
	*r = (*r > IMAGE_ROWS-FRM_CNSTR )? IMAGE_ROWS-FRM_CNSTR : *r;

	if ((*r == tmpr) && (*c == tmpc)) {
		return 0;
	}

	return 1;
}

/*
 *  FUNCTION: int roundFloatToInt(float f)
 *
 *  PARAMETERS: f, a number between -2^31 -1  and 2^31
 *
 *  DESCRIPTION: takes a float and rounds it to the nearest integer. Note that this is not statistically sound
 *
 *  RETURNS: an integer
 *
 */

int roundFloatToInt(float f)
{
	return (int)(f+0.5);
}

/*
 *  FUNCTION: Bresenhamline
 *
 *  PARAMETERS: (x0,y0), starting coordinates of line segment and
 * 		(x1,y1), ending coordinates of line segment
 *
 *  DESCRIPTION: draws line segment from start to end
 *
 *  RETURNS: void
 *
 * THANKS: http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *
 */
void Bresenhamline(int x0, int y0, int x1, int y1, bgr *ptrImage) {
	int Dx = x1 - x0;
	int Dy = y1 - y0;
	int steep = (abs(Dy) >= abs(Dx));
	int xstep, ystep;
	int TwoDy, TwoDyTwoDx,E;
	int xDraw, yDraw,x,y;
	if (steep) {
		SWAP(x0, y0);
		SWAP(x1, y1);
		// recompute Dx, Dy after swap
		Dx = x1 - x0;
		Dy = y1 - y0;
	}
	xstep = 1;
	if (Dx < 0) {
		xstep = -1;
		Dx = -Dx;
	}
	ystep = 1;
	if (Dy < 0) {
		ystep = -1;
		Dy = -Dy;
	}
	TwoDy = 2*Dy;
	TwoDyTwoDx = TwoDy - 2*Dx; // 2*Dy - 2*Dx
	E = TwoDy - Dx; //2*Dy - Dx
	y = y0;
	for (x = x0; x != x1; x += xstep) {
		if (steep) {
			xDraw = y;
			yDraw = x;
		} else {
			xDraw = x;
			yDraw = y;
		}
		// plot
		//plot(xDraw, yDraw);
		LCD_saturate(&yDraw,&xDraw);
		ptrImage[yDraw*IMAGE_COLUMNS+xDraw].green = 255; //paint line bright green

		// next
		if (E > 0) {
			E += TwoDyTwoDx; //E += 2*Dy - 2*Dx;
			y = y + ystep;
		} else {
			E += TwoDy; //E += 2*Dy;
		}
	}
	return;
}

