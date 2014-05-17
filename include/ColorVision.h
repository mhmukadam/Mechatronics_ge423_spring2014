#ifndef _COLORVISION_H_
#define _COLORVISION_H_

#define IMAGE_ROWS_SMALL 72
#define IMAGE_COLUMNS_SMALL 88

#define IMAGE_ROWS 144
#define IMAGE_COLUMNS 176

#define PIXEL_ROWS 288
#define PIXEL_COLS 352

#define PI	3.14159265358979

// only calculate the angle of an object if there are more than 10 pixels in the object  You can change this if you find it necessary.
#define NUMPIXELS_TO_CALC_ANGLE 10

#define	EVTCLR0 *(unsigned int *)0x01800040
#define	EVTCLR1 *(unsigned int *)0x01800044
#define	EVTCLR2 *(unsigned int *)0x01800048
#define	EVTCLR3 *(unsigned int *)0x0180004C


void init_ColorVision(void);


#endif /* _COLORVISION_H_ */
