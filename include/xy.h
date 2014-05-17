// macros
#define MIN(A,B)		(((A) < (B)) ? (A) : (B));
#define MAX(A,B)		(((A) > (B)) ? (A) : (B));

/* USER_XY.C */
int xy_control(float *_vref, float *_turn,float turn_thres, float x_pos,float y_pos,float x_desired,float y_desired,float thetaabs,float target_radius,float target_radius_near);
