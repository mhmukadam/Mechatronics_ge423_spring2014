 /*    
     ____________________________________________________
    |                                                   |
    |                                                   |
    |                                                   |
    |                                                   |
    |                    |            |                 |
    |                    |            |                 |
    |                    |            |                 |
    |                    |            |                 |
    |                    |____________|                 |
    |                                                   |
    |                                                   |
    |                                                   |
    |                                                   |
    |                                                   |
    |____________________     y       __________________|
                              ^
                              |
                              |    
                              |------> x
                              (0,0) Theta Right hand rule
*/


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

#include "ColorVision.h"
#include "xy.h"

#define HALFPI PI/2.0

float my_atanf(float dy, float dx)
{
	float ang;
    
	if (fabsf(dy) <= 0.001F) {
		if (dx >= 0.0F) {
			ang = 0.0F;
		} else {
			ang = PI;
		}
	} else if (fabsf(dx) <= 0.001F) {
		if (dy > 0.0F) {
			ang = HALFPI;
		} else {
			ang = -HALFPI;
		}
	} else {
		ang = atan2f(dy,dx);
	}
	return ang;
}

int xy_control(float *vref_forxy, float *turn_forxy,float turn_thres, float x_pos,float y_pos,float x_desired,float y_desired,float thetaabs,float target_radius,float target_radius_near)
{
	float dx,dy,alpha;
	float dist = 0.0F;
	float dir;
	float theta;
	int target_near = FALSE;
    float turnerror = 0;

		// calculate theta (current heading) between -PI and PI
	if (thetaabs > PI) {
		theta = thetaabs - 2.0*PI*floorf((thetaabs+PI)/(2.0*PI));
	} else if (thetaabs < -PI) {
		theta = thetaabs - 2.0*PI*ceilf((thetaabs-PI)/(2.0*PI));
	} else {
		theta = thetaabs;
	}

	dx = x_desired - x_pos;
	dy = y_desired - y_pos;
	dist = sqrtf( dx*dx + dy*dy );
	dir = 1.0F;

	// calculate alpha (trajectory angle) between -PI and PI
	alpha = my_atanf(dy,dx);

	// calculate turn error
	turnerror = theta - alpha;

	// check for shortest path
	if (fabsf(turnerror + 2.0*PI) < fabsf(turnerror)) turnerror += 2.0*PI;
	else if (fabsf(turnerror - 2.0*PI) < fabsf(turnerror)) turnerror -= 2.0*PI;

	if (dist < target_radius_near) {
		target_near = TRUE;
		// Arrived to the target's (X,Y)
		if (dist < target_radius) {
			dir = 0.0F;
			turnerror = 0.0F;
		} else {
			// if we overshot target, we must change direction. This can cause the robot to bounce back and forth when remaining at a point.
			if (fabsf(turnerror) > HALFPI) {
				dir = -dir;
			}            
			turnerror = 0;
		}
	} else {
		target_near = FALSE;
	}

	// vref is 1 tile/sec; but slower when close to target.  
	//*vref_forxy = dir*MIN(dist,1);
	*vref_forxy = dir;

    if (fabsf(*vref_forxy) > 0.0) {
        // if robot 1 tile away from target use a scaled KP value.  
        *turn_forxy = (*vref_forxy*2)*turnerror;
    } else {
        // normally use a Kp gain of 2
        *turn_forxy = 2*turnerror;
    }
    
    // This helps with unbalanced wheel slipping.  If turn value greater than turn_thres (I use 2) then just spin in place
	if (fabsf(*turn_forxy) > turn_thres) {
		*vref_forxy = 0;
	}
	return(target_near);
}

