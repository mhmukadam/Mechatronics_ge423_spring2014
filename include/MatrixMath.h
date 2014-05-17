#ifndef _USER_MATRIXMATH_H_
#define _USER_MATRIXMATH_H_

void Matrix3x3_Invert(float mat[3][3], float inverse[3][3]);

void Matrix3x3_Trans(float mat[3][3], float trans[3][3]);

void Matrix3x3_Mult(float A[3][3], float B[3][3], float X[3][3]);

void Matrix1x3_Mult(float A[1][3], float B[3][3], float X[1][3]);

void Matrix3x1_Mult(float A[3][3], float B[3][1], float X[3][1]);

void Matrix3x2_Mult(float A[3][2], float B[2][1], float X[3][1]);

void Matrix3x1_Add(float A[3][1], float B[3][1], float X[3][1], float scaleA, float scaleB);

void Matrix3x3_Add(float A[3][3], float B[3][3], float X[3][3], float scaleA, float scaleB);

#endif
