#ifndef _LADAR_H_
#define _LADAR_H_

#define LADAR_OUT_BUFFER_SIZE	30
#define LADAR_IN_BUFF_SIZE 1400
#define LADAR_FEET_PER_MM 0.0032808399
#define LADAR_MAX_READING 5700

//----------------------------------------------------------
// TO ADJUST NUMBER OF BLOCKS PRODUCED BY A LADAR SCAN:
//	If NUMDISTBLOCKS is changed, LADAR_MAX_DATA_SIZE
//	may need to be adjusted.  As an example, if you want
//	to scan from -120degrees to 120 degrees with 300 blocks,
//	you need to set LADAR_MAX_DATA_SIZE according to:
//
//		start = floor((-120.0+135.0)*768.0/270.0) = 42
//		finish = ceil((120.0+135.0)*768.0/270.0) = 726
//		clustersize = floor((finish-start)/NUMDISTBLOCKS) = 2
//		LADAR_MAX_DATA_SIZE = ceil((finish-start)/clustersize) = 342
//
//	Be conservative with your LADAR_MAX_DATA_SIZE.  For the example above,
//	LADAR_MAX_DATA_SIZE = 350 would be a good choice.
#define NUMDISTBLOCKS 228
#define LADAR_MAX_DATA_SIZE 250
//-----------------------------------------------------------

#define LADAR_INFINITY 8		// constants that indicate the slope
#define LADAR_HORIZONTAL 4
#define LADAR_MAXNUMCORNERS 20

// a pose (position and orientation) of the robot
typedef struct
{
	float x;		//in feet
	float y;		//in feet
	float theta;	// in radians between -PI and PI.  O radians is along the +x axis, PI/2 is the +y axis
} pose;

typedef struct bgr {
    unsigned char blue;
    unsigned char green;
    unsigned char red;
} bgr;

int LADAR_scan(float startSweepDeg, float endSweepDeg);
int LADAR_sendG(int startPoint, int endPoint, int clusterCount);
void LADAR_Get_and_Process_Scan(void);
int decode2byteLADAR(char* ptr, int previousValue);
void LADARdetections2xy (float* X, float*Y, const pose *ROBOTps, const pose *LADARps,
						 const float* Langles, const float* Ldistances, const int iReadings);
// function to return sign of a float
float signf(float n);

int init_LADAR(void);

void UpdateLCDwithLADAR(bgr *ptrImage, int use_world_ref_frame);
int roundFloatToInt(float);
int LCD_saturate(int *r,int *c);
void Bresenhamline(int x0, int y0, int x1, int y1, bgr *ptrImage);

#endif /* _LADAR_H_ */
