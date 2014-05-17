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
#include "COECSL_registers.h"

#include "mcbsp_com.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"
#include "LCDprintf.h"
#include "ladar.h"
#include "xy.h"
#include "MatrixMath.h"

#define FEETINONEMETER 3.28083989501312

extern EDMA3_CCRL_Regs *EDMA3_0_Regs;

volatile uint32_t index;

// test variables
extern float enc1;  // Left motor encoder
extern float enc2;  // Right motor encoder
extern float enc3;
extern float enc4;
extern float adcA0;  // ADC A0 - Gyro_X -400deg/s to 400deg/s  Pitch
extern float adcB0;  // ADC B0 - External ADC Ch4 (no protection circuit)
extern float adcA1;  // ADC A1 - Gyro_4X -100deg/s to 100deg/s  Pitch
extern float adcB1;  // ADC B1 - External ADC Ch1
extern float adcA2;  // ADC A2 -	Gyro_4Z -100deg/s to 100deg/s  Yaw
extern float adcB2;  // ADC B2 - External ADC Ch2
extern float adcA3;  // ADC A3 - Gyro_Z -400deg/s to 400 deg/s  Yaw
extern float adcB3;  // ADC B3 - External ADC Ch3
extern float adcA4;  // ADC A4 - Analog IR1
extern float adcB4;  // ADC B4 - USONIC1
extern float adcA5;  // ADC A5 -	Analog IR2
extern float adcB5;  // ADC B5 - USONIC2
extern float adcA6;  // ADC A6 - Analog IR3
extern float adcA7;  // ADC A7 - Analog IR4
extern float compass;
extern float switchstate;

float vref = 0;
float turn = 0;

int tskcount = 0;
char fromLinuxstring[LINUX_COMSIZE + 2];
char toLinuxstring[LINUX_COMSIZE + 2];

float VBvalue1 = 0;
float VBvalue2 = 0;
int new_VB_data = 0;

int newnavdata = 0;
float newvref = 0;
float newturn = 0;

extern sharedmemstruct *ptrshrdmem;

float x_pred[3][1] = {{0},{0},{0}};					// predicted state

//more kalman vars
float B[3][2] = {{1,0},{1,0},{0,1}};			// control input model
float u[2][1] = {{0},{0}};			// control input in terms of velocity and angular velocity
float Bu[3][1] = {{0},{0},{0}};	// matrix multiplication of B and u
float z[3][1];							// state measurement
float eye3[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// 3x3 identity matrix
float K[3][3] = {{1,0,0},{0,1,0},{0,0,1}};		// optimal Kalman gain
#define ProcUncert 0.0001
#define CovScalar 10
float Q[3][3] = {{ProcUncert,0,ProcUncert/CovScalar},
		{0,ProcUncert,ProcUncert/CovScalar},
		{ProcUncert/CovScalar,ProcUncert/CovScalar,ProcUncert}};	// process noise (covariance of encoders and gyro)
#define MeasUncert 1
float R[3][3] = {{MeasUncert,0,MeasUncert/CovScalar},
		{0,MeasUncert,MeasUncert/CovScalar},
		{MeasUncert/CovScalar,MeasUncert/CovScalar,MeasUncert}};	// measurement noise (covariance of LADAR)
float S[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance
float S_inv[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance matrix inverse
float P_pred[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// predicted covariance (measure of uncertainty for current position)
float temp_3x3[3][3];				// intermediate storage
float temp_3x1[3][1];				// intermediate storage
float ytilde[3][1];					// difference between predictions

// deadreckoning
float vel1 = 0,vel2 = 0;
float vel1old = 0,vel2old = 0;
float enc1old = 0,enc2old = 0;

// SETTLETIME should be an even number and divisible by 3
#define SETTLETIME 6000
int settlegyro = 0;
float gyro_zero = 0;
float gyro_angle = 0;
float old_gyro = 0;
float gyro_drift = 0;
float gyro = 0;
int gyro_degrees = 0;
float gyro_radians = 0.0;
float gyro_x = 0,gyro_y = 0;
float gyro4x_gain = 1;

int statePos = 0;	// index into robotdest
int robotdestSize = 8;	// number of positions to use out of robotdest
pose robotdest[8];	// array of waypoints for the robot

extern float newLADARdistance[LADAR_MAX_DATA_SIZE];  //in mm
extern float newLADARangle[LADAR_MAX_DATA_SIZE];		// in degrees
float LADARdistance[LADAR_MAX_DATA_SIZE];
float LADARangle[LADAR_MAX_DATA_SIZE];
extern pose ROBOTps;
extern pose LADARps;
extern float newLADARdataX[LADAR_MAX_DATA_SIZE];
extern float newLADARdataY[LADAR_MAX_DATA_SIZE];
float LADARdataX[LADAR_MAX_DATA_SIZE];
float LADARdataY[LADAR_MAX_DATA_SIZE];
extern int newLADARdata;

// Optitrack Variables
int trackableIDerror = 0;
int firstdata = 1;
volatile int new_optitrack = 0;
volatile float previous_frame = -1;
int frame_error = 0;
volatile float Optitrackdata[OPTITRACKDATASIZE];
pose OPTITRACKps;
float temp_theta = 0.0;
float tempOPTITRACK_theta = 0.0;
volatile int temp_trackableID = -1;
int trackableID = -1;
int errorcheck = 1;


//********************************************************************************************************//
//***********************//
#define MAXGOAL 6 //6 test: 1
#define GRIDROW 17
#define GRIDCOL 13
#define GRIDXN -6
#define GRIDXP 6
#define GRIDYN -4
#define GRIDYP 12
#define OBSTTHRES 0.1
#define OBSTCNTTHRES 40
#define MINGB 2.8 // 2.5
#define MAXPIXELS 8 //3
#define BLUEGOAL 5
#define ORANGEGOAL 6
#define GATEUPL 8
#define GATEUPR 7
#define GATEDOWNL 11.2
#define GATEDOWNR 4.1

float VREFGAIN = 1.3,TURNGAIN = 1.3;

// Variables for 2D maze generation with LADAR
int maze[GRIDROW][GRIDCOL];
int obstCount[GRIDROW][GRIDCOL];

// Variables for Astar
int runAstar = 0; // flag to run Astar
int goal_loc[2];
int visited[GRIDROW][GRIDCOL];

// Data structures and functions for Astar search
// Tree Nodes
struct STreeNode{
	int loc[2];
	int cost;
	int hn;
	int fn;
	struct STreeNode *parent, *next, *prev;
};

typedef struct STreeNode STreeNode;

// Priority queue
typedef struct {
	STreeNode *first;
	long int size;
} pque;

// Function declerations for data structures
void newSTreeNode(STreeNode *curr, int r, int c, STreeNode *p);
void newpque(pque *p);
void printque(pque *p);
STreeNode* pollpque(pque *p);
void add2pque(pque *p, STreeNode *n);

void newSTreeNode(STreeNode *curr, int r, int c, STreeNode *p) { //Constructor
	curr->loc[0] = r;
	curr->loc[1] = c;
	curr->parent = p;
	if (p != NULL) {
		curr->cost = p->cost + 1; //increment cost
		curr->hn = abs(goal_loc[0] - r) + abs(goal_loc[1] - c); //Manhattan distance heuristic
	}
	else {
		curr->cost = 0;
		curr->hn = 0;
	}
	curr->fn = curr->cost + curr->hn; //fn = gn + hn
	curr->next = NULL;
	curr->prev = NULL;
}

void newpque(pque *p) {//Constructor
	p->first = NULL;
	p->size = 0;
}

void add2pque(pque *p, STreeNode *n) {//add in ascending order
	int i=0;
	STreeNode *cond;
	if (p->first == NULL) {//if queue is empty
		p->first = n;
		(p->size)++;
		return;
	}
	else {
		cond = p->first;
		while (cond != NULL) {
			if (n->fn <= cond->fn) {
				n->next = cond;
				if (i == 0) {//if only one item in queue
					p->first = n;
					cond->prev = n;
					p->first->next = cond;
					p->first->prev = NULL;
					(p->size)++;
					return;
				}
				cond->prev->next = n;
				n->prev = cond->prev;
				cond->prev = n;
				(p->size)++;
				return;
			}
			else {
				if (cond->next == NULL) { //if last element
					cond->next = n;
					n->prev = cond;
					(p->size)++;
					return;
				}
				cond = cond->next;
				i = 1;
			}
		}
	}
}

STreeNode* pollpque(pque *p) {
	STreeNode *f;
	f = p->first;
	if (p->first->next != NULL) {
		p->first = p->first->next;
		p->first->prev = NULL;
	}
	else { //if only one element in the list
		p->first = NULL;
	}
	f->next = NULL;
	(p->size)--;
	return f;
}

// More Variables for Astar path planning
STreeNode *childs, *child, *start;
pque pq;
int path[100][2]; // (r,c)
float pathGC[100][2]; // (x,y)
int pathsize = 0;
int robotRow = 0, robotCol = 0; // to calculate start in GRID coordinates
int goalNo = 0;
int pathReady = 0;
int findingPath = 0;
int delayTime = 0;

int stateMachine = -1; // State of the robot

// Function declarations for path planning
void xy2gridrc(float x, float y, int *r, int *c);
void gridrc2xy(int r, int c, float *x, float *y);
int updateMaze(void);

// Variables for golf ball detection and collection
extern char colortoggle;// 0 for blue, 1 for orange
extern volatile float bobject_x, oobject_x;
extern volatile float bobject_y, oobject_y;
extern volatile int bnumpels, onumpels;
extern volatile int new_coordata;
float kp_vision = 0.05;
float dcoeff[4]={5.8801161742e-05,-4.2015516280e-03,1.1662055486e-01,6.9514845250e-01};
float bdist = 0.0, odist = 0.0;
float brow = 0.0, orow = 0.0;
float lgate = GATEDOWNL, rgate = GATEDOWNR;
float cbx, cby; // collected ball (x,y)
float cbc; // collected ball color
int ballcnt = 21; // Using data array from 21
//***********************//

//***********************//
#define DATASIZE 36

int rows[GRIDROW];
int patharray[50][50];
int DSLen = 0;
char dataString[256];
float new_DataArr[DATASIZE];
float DataArr[DATASIZE];
int newMatlab_data = 0;
//***********************//

//********************************************************************************************************//

pose UpdateOptitrackStates(pose localROBOTps, int * flag);


void ComWithLinux(void) {

	int i = 0;
	TSK_sleep(100);

	while(1) {

		//**********************************************************************************************************//
		BCACHE_inv((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

		if (GET_DATA_FROM_LINUX) {

			if (newnavdata == 0) {
				newvref = ptrshrdmem->Floats_to_DSP[0];
				newturn = ptrshrdmem->Floats_to_DSP[1];
				newnavdata = 1;
			}

			CLR_DATA_FROM_LINUX;

		}

		if (GET_VBDATA_FROM_LINUX) {

			if (ptrshrdmem->DSPRec_size > 256) ptrshrdmem->DSPRec_size = 256;
			for (i=0;i<ptrshrdmem->DSPRec_size;i++) {
				fromLinuxstring[i] = ptrshrdmem->DSPRec_buf[i];
			}
			fromLinuxstring[i] = '\0';

			if (new_VB_data == 0) {
				sscanf(fromLinuxstring,"%f,%f",&VREFGAIN,&TURNGAIN);
				new_VB_data = 1;
			}

			CLR_VBDATA_FROM_LINUX;

			if (GET_VBDATA_TO_LINUX) {

				// Default

				if (newMatlab_data == 1) {

					for (i=0;i<DATASIZE;i++) {
						new_DataArr[i] = DataArr[i];
					}

					newMatlab_data = 0;
				}

				DSLen=sprintf(dataString,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",new_DataArr[0],new_DataArr[1],new_DataArr[2],new_DataArr[3],new_DataArr[4],new_DataArr[5],new_DataArr[6],new_DataArr[7],new_DataArr[8],new_DataArr[9],new_DataArr[10],new_DataArr[11],new_DataArr[12],new_DataArr[13],new_DataArr[14],new_DataArr[15],new_DataArr[16],new_DataArr[17],new_DataArr[18],new_DataArr[19],new_DataArr[20],new_DataArr[21],new_DataArr[22],new_DataArr[23],new_DataArr[24],new_DataArr[25],new_DataArr[26],new_DataArr[27],new_DataArr[28],new_DataArr[29],new_DataArr[30],new_DataArr[31],new_DataArr[32],new_DataArr[33],new_DataArr[34],new_DataArr[35]);

				ptrshrdmem->DSPSend_size=sprintf(toLinuxstring,"%d,%s",DSLen,dataString);

				for (i=0;i<ptrshrdmem->DSPSend_size;i++) {
					ptrshrdmem->DSPSend_buf[i] = toLinuxstring[i];
				}

				// Flush or write back source
				BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

				CLR_VBDATA_TO_LINUX;

			}


		}
		//********************************************************************************************************//


		if (GET_DATAFORFILE_TO_LINUX) {
			int j = 0, l = 0, m = 0, k = 0;
			// First make sure all scratch elements are zero
			for (i=0;i<500;i++) {
				ptrshrdmem->scratch[i] = 0;
			}


			//*****************************************************************************************************//
			// build 17 element array of maze
			for (i=0;i<GRIDROW;i++) {
				rows[i]=0;
				for (j=0;j<GRIDCOL;j++) {
					rows[i] |= maze[i][j]<<j;
				}
			}

			// updating path of the robot
			for (i=0;i<pathsize - 1;i++) {
				for (j=0;j<pathsize - 1;j++) {
					patharray[i][j] = patharray[path[i][0]][path[i][1]];
				}
			}

			//transmit the row elements to scratch
			for (l=0;l<GRIDROW;l++){
				ptrshrdmem->scratch[k] = (float)rows[l];
				k++;
			}

			//transmit the path array elements to scratch
			for (l=0;l<pathsize - 1;l++){
				for (m=0;m<pathsize - 1;m++){
					ptrshrdmem->scratch[k] = (float)patharray[l][m];
					k++;
				}
			}

			//*****************************************************************************************************//
			// Flush or write back source
			BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

			CLR_DATAFORFILE_TO_LINUX;
		}



		tskcount++;
		TSK_sleep(40);
	}


}


/*
 *  ======== main ========
 */
Void main()
{

	int i = 0, j = 0;

	// unlock the system config registers.
	SYSCONFIG->KICKR[0] = KICK0R_UNLOCK;
	SYSCONFIG->KICKR[1] = KICK1R_UNLOCK;

	SYSCONFIG1->PUPD_SEL |= 0x10000000;  // change pin group 28 to pullup for GP7[12/13] (LCD switches)

	// Initially set McBSP1 pins as GPIO ins
	CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[1], 0x88888880);  // This is enabling the McBSP1 pins

	CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
	SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13 
	CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
	SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13


	//Rick added for LCD DMA flagging test
	GPIO_setDir(GPIO_BANK0, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setOutput(GPIO_BANK0, GPIO_PIN8, OUTPUT_HIGH);

	GPIO_setDir(GPIO_BANK0, GPIO_PIN0, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN1, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN2, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN3, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN4, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN5, GPIO_INPUT);  
	GPIO_setDir(GPIO_BANK0, GPIO_PIN6, GPIO_INPUT);

	GPIO_setDir(GPIO_BANK7, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN9, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN10, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN11, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN12, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN13, GPIO_INPUT); 

	GPIO_setOutput(GPIO_BANK7, GPIO_PIN8, OUTPUT_HIGH);  
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN9, OUTPUT_HIGH);
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN10, OUTPUT_HIGH);
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN11, OUTPUT_HIGH);  

	CLRBIT(SYSCONFIG->PINMUX[13], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[13], 0x88888811); //Set GPIO 6.8-13 to GPIOs and IMPORTANT Sets GP6[15] to /RESETOUT used by PHY, GP6[14] CLKOUT appears unconnected

	//#warn GP6.15 is also connected to CAMERA RESET This is a Bug in my board design Need to change Camera Reset to different IO.

	GPIO_setDir(GPIO_BANK6, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN9, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN10, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN11, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN12, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN13, GPIO_INPUT);   


	// on power up wait until Linux has initialized Timer1
	while ((T1_TGCR & 0x7) != 0x7) {
		for (index=0;index<50000;index++) {}  // small delay before checking again

	}

	USTIMER_init();

	// Turn on McBSP1
	EVMOMAPL138_lpscTransition(PSC1, DOMAIN0, LPSC_MCBSP1, PSC_ENABLE);

	// If Linux has already booted It sets a flag so no need to delay
	if ( GET_ISLINUX_BOOTED == 0) {
		USTIMER_delay(4*DELAY_1_SEC);  // delay allowing Linux to partially boot before continuing with DSP code
	}

	// init the us timer and i2c for all to use.
	I2C_init(I2C0, I2C_CLK_100K);
	init_ColorVision();	
	init_LCD_mem(); // added rick

	EVTCLR0 = 0xFFFFFFFF;
	EVTCLR1 = 0xFFFFFFFF;
	EVTCLR2 = 0xFFFFFFFF;
	EVTCLR3 = 0xFFFFFFFF;	

	init_DMA();
	init_McBSP();

	init_LADAR();

	CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[1], 0x22222220);  // This is enabling the McBSP1 pins

	CLRBIT(SYSCONFIG->PINMUX[5], 0x00FF0FFF);
	SETBIT(SYSCONFIG->PINMUX[5], 0x00110111);  // This is enabling SPI pins

	CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
	SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13 
	CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
	SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13

	init_LCD();

	LADARps.x = 3.5/12; // 3.5/12 for front mounting
	LADARps.y = 0;
	LADARps.theta = 1;  // not inverted

	OPTITRACKps.x = 0;
	OPTITRACKps.y = 0;
	OPTITRACKps.theta = 0;

	for(i = 0;i<LADAR_MAX_DATA_SIZE;i++)
	{ LADARdistance[i] = LADAR_MAX_READING; } //initialize all readings to max value.

	//********************************************************************************************************//
	
	// ROBOTps will be updated by Optitrack during gyro calibration
	// TODO: specify the starting position of the robot
	ROBOTps.x = 0; //0	3		//the estimate in array form (useful for matrix operations) (in world coordinates)
	ROBOTps.y = -1; //-1 9
	ROBOTps.theta = 0;  // was -PI: need to flip OT ground plane to fix this
	x_pred[0][0] = ROBOTps.x; //estimate in structure form (useful elsewhere)
	x_pred[1][0] = ROBOTps.y;
	x_pred[2][0] = ROBOTps.theta;

	// TODO: defined destinations that moves the robot around and outside the course (in coordinates of GRIDmaze) x==col,y==row
	robotdest[0].x = 1; robotdest[0].y = 15; // Check point 1 test (3,3) else 1,15
	robotdest[1].x = 9;	robotdest[1].y = 5; // Check point 2
	robotdest[2].x = 3;	robotdest[2].y = 5; // Check point 3
	robotdest[3].x = 11; robotdest[3].y = 15; // Check point 4
	robotdest[4].x = 6; robotdest[4].y = 1; // Check point 5
	robotdest[5].x = 4; robotdest[5].y = 16; // Blue golf ball drop point
	robotdest[6].x = 8; robotdest[6].y = 16; // Green golf ball drop point

	for (i=0;i<GRIDROW;i++) {
		for (j=0;j<GRIDCOL;j++) {
			maze[i][j] = 0;
			obstCount[i][j] = 0;
		}
	}

	childs = (STreeNode *)0xc4000000;
	goal_loc[0] = robotdest[goalNo].y;
	goal_loc[1] = robotdest[goalNo].x;

	for(i = 21; i < DATASIZE; i++){
		DataArr[i] = -8;
	}

	//********************************************************************************************************//

	// flag pins
	GPIO_setDir(IMAGE_TO_LINUX_BANK, IMAGE_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(OPTITRACKDATA_FROM_LINUX_BANK, OPTITRACKDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATA_TO_LINUX_BANK, DATA_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATA_FROM_LINUX_BANK, DATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATAFORFILE_TO_LINUX_BANK, DATAFORFILE_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(VBDATA_FROM_LINUX_BANK, VBDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(VBDATA_TO_LINUX_BANK, VBDATA_TO_LINUX_FLAG, GPIO_OUTPUT);


	CLR_OPTITRACKDATA_FROM_LINUX;  // Clear = tell linux DSP is ready for new Opitrack data
	CLR_DATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new data
	CLR_DATAFORFILE_TO_LINUX;  // Clear = linux not requesting data
	SET_DATA_TO_LINUX;  // Set = put float array data into shared memory for linux
	SET_IMAGE_TO_LINUX;  // Set = put image into shared memory for linux
	CLR_VBDATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new VB data
	SET_VBDATA_TO_LINUX;  // Set = put VB char data into shared memory for linux

	// clear all possible EDMA
	EDMA3_0_Regs->SHADOW[1].ICR = 0xFFFFFFFF;
}

long timecount= 0;
int whichled = 0;

// This SWI is Posted after each set of new data from the F28335
void RobotControl(void) {

	int newOPTITRACKpose = 0;
	int i = 0;
	int j=0;

	if (0==(timecount%1000)) {
		switch(whichled) {
		case 0:
			SETREDLED;
			CLRBLUELED;
			CLRGREENLED;
			whichled = 1;
			break;
		case 1:
			CLRREDLED;
			SETBLUELED;
			CLRGREENLED;
			whichled = 2;
			break;
		case 2:
			CLRREDLED;
			CLRBLUELED;
			SETGREENLED;
			whichled = 0;
			break;
		default:
			whichled = 0;
			break;
		}
	}

	if (GET_OPTITRACKDATA_FROM_LINUX) {

		if (new_optitrack == 0) {
			for (i=0;i<OPTITRACKDATASIZE;i++) {
				Optitrackdata[i] = ptrshrdmem->Optitrackdata[i];
			}
			new_optitrack = 1;
		}

		CLR_OPTITRACKDATA_FROM_LINUX;

	}

	if (new_optitrack == 1) {
		OPTITRACKps = UpdateOptitrackStates(ROBOTps, &newOPTITRACKpose);
		new_optitrack = 0;
	}

	// using 400deg/s gyro
	gyro = adcA3*3.0/4096.0;
	if (settlegyro < SETTLETIME) {
		settlegyro++;
		if (settlegyro < (SETTLETIME/3)) {
			// do nothing
		} else if (settlegyro < (2*SETTLETIME/3)) {
			gyro_zero = gyro_zero + gyro/(SETTLETIME/3);
		} else {
			gyro_drift += (((gyro-gyro_zero) + old_gyro)*.0005)/(SETTLETIME/3);
			old_gyro = gyro-gyro_zero;
		}
		if(settlegyro%500 == 0) {
			LCDPrintfLine(1,"Cal Gyro -- %.1fSecs", (float)(SETTLETIME - settlegyro)/1000.0 );
			LCDPrintfLine(2,"");
		}

		newOPTITRACKpose = 0;

		SetRobotOutputs(0,0,0,0,0,0,0,0,0,0);
	} else {

		gyro_angle = gyro_angle - ((gyro-gyro_zero) + old_gyro)*.0005 + gyro_drift; 
		old_gyro = gyro-gyro_zero;
		gyro_radians = (gyro_angle * (PI/180.0)*400.0*gyro4x_gain);

		// Kalman filtering
		vel1 = (enc1 - enc1old)/(193.0*0.001);	// calculate actual velocities
		vel2 = (enc2 - enc2old)/(193.0*0.001);
		if (fabsf(vel1) > 10.0) vel1 = vel1old;	// check for encoder roll-over should never happen
		if (fabsf(vel2) > 10.0) vel2 = vel2old;
		enc1old = enc1;	// save past values
		enc2old = enc2;
		vel1old = vel1;
		vel2old = vel2;

		// Step 0: update B, u
		B[0][0] = cosf(ROBOTps.theta)*0.001;
		B[1][0] = sinf(ROBOTps.theta)*0.001;
		B[2][1] = 0.001;
		u[0][0] = 0.5*(vel1 + vel2);	// linear velocity of robot
		u[1][0] = (gyro-gyro_zero)*(PI/180.0)*400.0*gyro4x_gain;	// angular velocity in rad/s (negative for right hand angle)

		// Step 1: predict the state and estimate covariance
		Matrix3x2_Mult(B, u, Bu);					// Bu = B*u
		Matrix3x1_Add(x_pred, Bu, x_pred, 1.0, 1.0); // x_pred = x_pred(old) + Bu
		Matrix3x3_Add(P_pred, Q, P_pred, 1.0, 1.0);	// P_pred = P_pred(old) + Q
		// Step 2: if there is a new measurement, then update the state
		if (1 == newOPTITRACKpose) {
			newOPTITRACKpose = 0;
			z[0][0] = OPTITRACKps.x;	// take in the LADAR measurement
			z[1][0] = OPTITRACKps.y;
			// fix for OptiTrack problem at 180 degrees
			if (cosf(ROBOTps.theta) < -0.99) {
				z[2][0] = ROBOTps.theta;
			}
			else {
				z[2][0] = OPTITRACKps.theta;
			}
			// Step 2a: calculate the innovation/measurement residual, ytilde
			Matrix3x1_Add(z, x_pred, ytilde, 1.0, -1.0);	// ytilde = z-x_pred
			// Step 2b: calculate innovation covariance, S
			Matrix3x3_Add(P_pred, R, S, 1.0, 1.0);							// S = P_pred + R
			// Step 2c: calculate the optimal Kalman gain, K
			Matrix3x3_Invert(S, S_inv);
			Matrix3x3_Mult(P_pred,  S_inv, K);								// K = P_pred*(S^-1)
			// Step 2d: update the state estimate x_pred = x_pred(old) + K*ytilde
			Matrix3x1_Mult(K, ytilde, temp_3x1);
			Matrix3x1_Add(x_pred, temp_3x1, x_pred, 1.0, 1.0);
			// Step 2e: update the covariance estimate   P_pred = (I-K)*P_pred(old)
			Matrix3x3_Add(eye3, K, temp_3x3, 1.0, -1.0);
			Matrix3x3_Mult(temp_3x3, P_pred, P_pred);
		}	// end of correction step

		// set ROBOTps to the updated and corrected Kalman values.
		ROBOTps.x = x_pred[0][0];
		ROBOTps.y = x_pred[1][0];
		ROBOTps.theta = x_pred[2][0];


		//********************************************************************************************************//

		//***********************//
		// Reset flag to get new vision data
		if (new_coordata == 1) {
			new_coordata = 0;
		}

		// Finding distance to a blob if one exists
		if (bnumpels > MAXPIXELS) {
			brow = bobject_y + IMAGE_ROWS/2;
			bdist = dcoeff[0]*brow*brow*brow + dcoeff[1]*brow*brow + dcoeff[2]*brow + dcoeff[3];
		}
		else bdist = 50.0;
		if (onumpels > MAXPIXELS) {
			orow = oobject_y + IMAGE_ROWS/2;
			odist = dcoeff[0]*orow*orow*orow + dcoeff[1]*orow*orow + dcoeff[2]*orow + dcoeff[3];
		}
		else odist = 50.0;

		if (newLADARdata == 1) {
			newLADARdata = 0;
			for (i=0;i<228;i++) {
				LADARdistance[i] = newLADARdistance[i];
				LADARangle[i] = newLADARangle[i];
				LADARdataX[i] = newLADARdataX[i];
				LADARdataY[i] = newLADARdataY[i];
			}
			if (updateMaze()) {
				// New obstacle has been recorded - recalculate the path
				SWI_post(&SWI_Astar);
			}
		}

		// Changing stateMachine of the robot
		switch (stateMachine) {
		case 0: // Reaching checkpoints
			// walk on the calculated path
			if (statePos < pathsize && pathReady == 1) {
				if(xy_control(&vref, &turn, 1.0, ROBOTps.x, ROBOTps.y, pathGC[statePos][0], pathGC[statePos][1], ROBOTps.theta, 0.25, 0.25)) {
					statePos++;
				}
			}
			if (goalNo < MAXGOAL && statePos == pathsize && pathReady == 1) { // Use new goal and calculate new path
				goalNo++;
			}
			stateMachine = -1;
			if (goalNo == MAXGOAL && statePos == pathsize) { // final destination reached, stop moving
				vref = 0.0;
				turn = 0.0;
				stateMachine = 3; //3, testing = 5
			}
			break;
		case 1: // collect blue golf ball
			pathReady = 0;
			statePos = 0;
			pathsize = 0;
			pathReady = 0;
			findingPath = 0;
			if (bnumpels > MAXPIXELS && bdist < MINGB) {
				turn = -1*(bobject_x)*kp_vision;
				vref = 0;
				if (abs(turn) < 0.001) {
					vref = 1;
					lgate = GATEUPL;
				}
			}
			else {
				delayTime++;
				vref = 1;
				turn = 0;
				if (delayTime >= 400) {
					vref = 0.0;
					turn = 0.0;
					delayTime = 0;
					stateMachine = -1;
					lgate = GATEDOWNL;
					cbx = ROBOTps.x + 0.667*cos(ROBOTps.theta) - 0.208*sin(ROBOTps.theta);
					cby = ROBOTps.y + 0.208*cos(ROBOTps.theta) + 0.667*sin(ROBOTps.theta);
					cbc = 0;
					DataArr[ballcnt] = cbx;
					ballcnt++;
					DataArr[ballcnt] = cby;
					ballcnt++;
					DataArr[ballcnt] = cbc;
					ballcnt++;
				}
			}
			break;
		case 2: // collect orange golf ball
			pathReady = 0;
			statePos = 0;
			pathsize = 0;
			pathReady = 0;
			findingPath = 0;
			if (onumpels > MAXPIXELS && odist < MINGB) {
				turn = -1*(oobject_x)*kp_vision;
				vref = 0;
				if (abs(turn) < 0.001) {
					vref = 1;
					rgate = GATEUPR;
				}
			}
			else {
				delayTime++;
				vref = 1;
				turn = 0;
				if (delayTime >= 400) {
					vref = 0.0;
					turn = 0.0;
					delayTime = 0;
					stateMachine = -1;
					rgate = GATEDOWNR;
					cbx = ROBOTps.x + 0.667*cos(ROBOTps.theta) + 0.208*sin(ROBOTps.theta);
					cby = ROBOTps.y - 0.208*cos(ROBOTps.theta) + 0.667*sin(ROBOTps.theta);
					cbc = 1;
					DataArr[ballcnt] = cbx;
					ballcnt++;
					DataArr[ballcnt] = cby;
					ballcnt++;
					DataArr[ballcnt] = cbc;
					ballcnt++;
				}
			}
			break;
		case 3: // deposit blue golf ball(s)
			stateMachine = 3;
			if (delayTime <= 1000) { // final destination reached, stop moving
				delayTime++;
				vref = 0.0;
				turn = 0.0;
				lgate = GATEUPL;
			}
			else {
				delayTime++;
				vref = -1;
				turn = 0;
				if (delayTime >= 2000) {
					vref = 0.0;
					turn = 0.0;
					delayTime = 0;
					stateMachine = 4;
					lgate = GATEDOWNL;
					maze[(int)robotdest[BLUEGOAL].y][(int)robotdest[BLUEGOAL].x] = 1;
					goal_loc[0] = robotdest[ORANGEGOAL].y;
					goal_loc[1] = robotdest[ORANGEGOAL].x;
					SWI_post(&SWI_Astar);
					pathReady = 0;
				}
			}
			break;
		case 4: // deposit orange golf ball(s)
			stateMachine = 4;
			if (statePos < pathsize && pathReady == 1) {
				if(xy_control(&vref, &turn, 1.0, ROBOTps.x, ROBOTps.y, pathGC[statePos][0], pathGC[statePos][1], ROBOTps.theta, 0.25, 0.25)) {
					statePos++;
				}
			}
			if (statePos == pathsize && delayTime <= 500 && pathReady == 1) { // final destination reached, stop moving
				delayTime++;
				vref = 0.0;
				turn = 0.0;
				rgate = GATEUPR;
			}
			if (delayTime > 500) {
				delayTime++;
				vref = -1;
				turn = 0;
				if (delayTime >= 1500) {
					vref = 0.0;
					turn = 0.0;
					delayTime = 0;
					stateMachine = 5;
					rgate = GATEDOWNR;
				}
			}
			break;
		case 5: // End program
			vref = 0.0;
			turn = 0.0;
			lgate = GATEDOWNL;
			rgate = GATEDOWNR;
			break;
		default:
			if (timecount < 1000) {
				vref = 0.0;
				turn = 0.0;
				stateMachine = -1;
			}
			else {
				if (bdist < MINGB) stateMachine = 1;
				else if (odist < MINGB) stateMachine = 2;
				else {
					stateMachine = 0;
					if (goalNo < MAXGOAL && statePos == pathsize && findingPath == 0) { // Use new goal and calculate new path
						goal_loc[0] = robotdest[goalNo].y;
						goal_loc[1] = robotdest[goalNo].x;
						SWI_post(&SWI_Astar);
					}
				}
			}
			break;
		}

		// Printing
		if ((timecount%200)==0) {
			LCDPrintfLine(1,"v=%.1f,t=%.1f,x=%.1f,y=%.1f",vref,turn,cbx,cby);
			LCDPrintfLine(2,"x=%.1f,y=%.1f,t=%.1f",ROBOTps.x,ROBOTps.y,ROBOTps.theta);

		}

		//***********************//

		//***********************//
		// updating rows array for the course map
		for (i=0;i<GRIDROW;i++) {
			rows[i]=0;
			for (j=0;j<GRIDCOL;j++) {
				rows[i] |= maze[i][j]<<j;
			}
		}

		// updating path of the robot
		for (i=0;i<pathsize - 1;i++) {
			patharray[i][j] = patharray[path[i][0]][path[i][1]];
		}

		// sending to matlab
		if(newMatlab_data==0){
			for(j=0;j<17;j++)
			{
				DataArr[j]=rows[j]; // course map
			}

			// x and y position of the robot
			DataArr[17]=ROBOTps.x;
			DataArr[18]=ROBOTps.y;
			DataArr[20]=ROBOTps.theta;

			//path length
			DataArr[19] = pathsize;

			newMatlab_data=1;
		}
		//******************************//


		//********************************************************************************************************//

		SetRobotOutputs(VREFGAIN*vref,TURNGAIN*turn,lgate,rgate,0,0,0,0,0,0);

		timecount++;
	}
}

pose UpdateOptitrackStates(pose localROBOTps, int * flag) {

	pose localOPTITRACKps;

	// Check for frame errors / packet loss
	if (previous_frame == Optitrackdata[OPTITRACKDATASIZE-1]) {
		frame_error++;
	}
	previous_frame = Optitrackdata[OPTITRACKDATASIZE-1];

	// Set local trackableID if first receive data
	if (firstdata){
		//trackableID = (int)Optitrackdata[OPTITRACKDATASIZE-1]; // removed to add new trackableID in shared memory
		trackableID = Optitrackdata[OPTITRACKDATASIZE-2];
		firstdata = 0;
	}

	// Check if local trackableID has changed - should never happen
	if (trackableID != Optitrackdata[OPTITRACKDATASIZE-2]) {
		trackableIDerror++;
		// do some sort of reset(?)
	}

	// Save position and yaw data
	if (isnan(Optitrackdata[0]) != 1) {  // this checks if the position data being received contains NaNs
		// check if x,y,yaw all equal 0.0 (almost certainly means the robot is untracked)
		if ((Optitrackdata[0] != 0.0) && (Optitrackdata[1] != 0.0) && (Optitrackdata[2] != 0.0)) {
			// save x,y
			// adding 2.5 so everything is shifted such that optitrack's origin is the center of the arena (while keeping all coordinates positive)
			localOPTITRACKps.x = Optitrackdata[0]*FEETINONEMETER; // was 2.5 for size = 5
			localOPTITRACKps.y = -1.0*Optitrackdata[1]*FEETINONEMETER+4.0;

			// make this a function
			temp_theta = fmodf(localROBOTps.theta,(float)(2*PI));//(theta[trackableID]%(2*PI));
			tempOPTITRACK_theta = Optitrackdata[2];
			if (temp_theta > 0) {
				if (temp_theta < PI) {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA > 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta > (PI/2)) {
							// THETA > 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA > 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA > 0, kal in QIII, OT in QIII
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta > (3*PI/2)) {
							// THETA > 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA > 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = (floorf((localROBOTps.theta)/((float)(2.0*PI))))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			} else {
				if (temp_theta > -PI) {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA < 0, kal in QIII/IV, OT in QIII/IV
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta < (-PI/2)) {
							// THETA < 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA < 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA < 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta < (-3*PI/2)) {
							// THETA < 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA < 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			}
			*flag = 1;
		}
	}
	return localOPTITRACKps;
}

//********************************************************************************************************//

void xy2gridrc(float x, float y, int *r, int *c) {
	*c = (int)x + 6;
	*r = 12 - (int)y;
}

void gridrc2xy(int r, int c, float *x, float *y) {
	*x = (float)(c - 6);
	*y = (float)(12 - r);
}

int updateMaze(void) {
	float lx = 0.0, ly = 0.0;
	int i = 0, j = 0, r = 0, c = 0;
	int update = 0;
	for (i=0;i<228;i++) {
		lx = LADARdataX[i];
		ly = LADARdataY[i];
		if (lx < GRIDXN || lx > GRIDXP || ly < GRIDYN || ly > GRIDYP) continue;
		else if (fabs(lx - round(lx)) <= OBSTTHRES && fabs(lx - round(lx)) <= OBSTTHRES) {
			xy2gridrc(round(lx),round(ly),&r,&c);
			obstCount[r][c]++;
		}
	}
	for (i=0;i<GRIDROW;i++) {
		for (j=0;j<GRIDCOL;j++) {
			if (obstCount[i][j] >= OBSTCNTTHRES) {
				obstCount[i][j] = OBSTCNTTHRES;
				maze[i][j] = 1;
				update = 1;
			}
		}
	}
	return update;
}

// This SWI is posted every time a new obstacle is detected or a new goal has to be reached
void Astar(void) {
	STreeNode *n;

	child = childs;
	start = child;
	statePos = 0;
	pathsize = 0;
	pathReady = 0;
	findingPath = 1;
	xy2gridrc((round(ROBOTps.x)),round(ROBOTps.y),&robotRow,&robotCol);
	newSTreeNode(start,robotRow,robotCol,NULL);

	int i=0,j=0,r=0,c=0,end=0;
	for (i=0;i<GRIDROW;i++) {
		for (j=0;j<GRIDCOL;j++) {
			visited[i][j] = 0;
		}
	}
	newpque(&pq);

	add2pque(&pq,start);
	visited[start->loc[0]][start->loc[1]] = 1;
	while (pq.size != 0) {
		n = pollpque(&pq);
		r = n->loc[0];
		c = n->loc[1];
		if (n->loc[0] == goal_loc[0] && n->loc[1] == goal_loc[1]) {
			pathsize=0;
			// Go back up the tree to save path
			while (n->parent != NULL) {
				path[pathsize][0] = n->loc[0];
				path[pathsize][1] = n->loc[1];
				pathsize++;
				n = n->parent;
			}
			// Reverse the path array
			end = pathsize-1;
			for (i=0;i<=end/2+1;i++) {
				r = path[i][0];
				c = path[i][1];
				path[i][0] = path[end][0];
				path[i][1] = path[end][1];
				path[end][0] = r;
				path[end][1] = c;
				end--;
			}
			// Convert to global coordinates
			for (i=0;i<pathsize;i++) {
				gridrc2xy(path[i][0],path[i][1],&pathGC[i][0],&pathGC[i][1]);
			}
			pathReady = 1;
			findingPath = 0;
			break;
		}
		else {
            // Check topleft
			if (r != 0 && c != 0 && visited[r-1][c-1] == 0 && maze[r-1][c-1] == 0) {
				visited[r-1][c-1] = 1;
				child++;
				newSTreeNode(child,r-1,c-1,n);
				add2pque(&pq,child);
			}
			// Check bottomleft
			if (r != (GRIDROW-1) && c != 0 && visited[r+1][c-1] == 0 && maze[r+1][c-1] == 0) {
				visited[r+1][c-1] = 1;
				child++;
				newSTreeNode(child,r+1,c-1,n);
				add2pque(&pq,child);
			}
			// Check topright
			if (r != 0 && c != (GRIDCOL-1) && visited[r-1][c+1] == 0 && maze[r-1][c+1] == 0) {
				visited[r-1][c+1] = 1;
				child++;
				newSTreeNode(child,r-1,c+1,n);
				add2pque(&pq,child);
			}
			// Check bottomright
			if (r != (GRIDROW-1) && c != (GRIDCOL-1) && visited[r+1][c+1] == 0 && maze[r+1][c+1] == 0) {
				visited[r+1][c+1] = 1;
				child++;
				newSTreeNode(child,r+1,c+1,n);
				add2pque(&pq,child);
			}
			// Check left
			if (c != 0 && visited[r][c-1] == 0 && maze[r][c-1] == 0) {
				visited[r][c-1] = 1;
				child++;
				newSTreeNode(child,r,c-1,n);
				add2pque(&pq,child);
			}
			// Check top
			if (r != 0 && visited[r-1][c] == 0 && maze[r-1][c] == 0) {
				visited[r-1][c] = 1;
				child++;
				newSTreeNode(child,r-1,c,n);
				add2pque(&pq,child);
			}
			// Check right
			if (c != (GRIDCOL-1) && visited[r][c+1] == 0 && maze[r][c+1] == 0) {
				visited[r][c+1] = 1;
				child++;
				newSTreeNode(child,r,c+1,n);
				add2pque(&pq,child);
			}
			// Check bottom
			if (r != (GRIDROW-1) && visited[r+1][c] == 0 && maze[r+1][c] == 0) {
				visited[r+1][c] = 1;
				child++;
				newSTreeNode(child,r+1,c,n);
				add2pque(&pq,child);
			}
		}
	}
}

//********************************************************************************************************//
