/*******************************************************************
 *
 *    DESCRIPTION: This file does matrix math for 3x3 matrices				 
 *
 *    AUTHOR: Todd Baxter, Aaron Becker
 *
 *    HISTORY: version beta-1.0
 *
 *******************************************************************/

/** include files **/
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
#include "MatrixMath.h"


void Matrix3x3_Invert(float mat[3][3], float inverse[3][3])
{
    float det, invDet;
    int i,j;

    inverse[0][0] = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
    inverse[1][0] = mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2];
    inverse[2][0] = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];

    det = mat[0][0] * inverse[0][0] + mat[0][1] * inverse[1][0] + mat[0][2] * inverse[2][0];

    if(det == 0)
    det = 1;

    invDet = 1.0f / det;

    inverse[0][1] = mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2];
    inverse[0][2] = mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1];
    inverse[1][1] = mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0];
    inverse[1][2] = mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2];
    inverse[2][1] = mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1];
    inverse[2][2] = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];

    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            inverse[i][j]*= invDet;
        }
    }
    return; 
}


void Matrix3x3_Trans(float mat[3][3], float trans[3][3])
{
    int i,j;
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            trans[i][j] = mat[j][i];
        }
    }
    return; 
}

void Matrix3x3_Mult(float A[3][3], float B[3][3], float X[3][3])
{
    int i,j;
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            X[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        }
    }
    return; 
}

void Matrix1x3_Mult(float A[1][3], float B[3][3], float X[1][3])
{
    int i;
    for(i=0;i<3;i++)
    {
        X[0][i] = A[0][0]*B[0][i] + A[0][1]*B[1][i] + A[0][2]*B[2][i];
    }
    return; 
}

// 3x3*3x1 = 3x1
void Matrix3x1_Mult(float A[3][3], float B[3][1], float X[3][1])
{
    int i;
    for(i=0;i<3;i++)
    {
        X[i][0] = A[i][0]*B[0][0] + A[i][1]*B[1][0] + A[i][2]*B[2][0];
    }
    return; 
}

void Matrix3x2_Mult(float A[3][2], float B[2][1], float X[3][1])
{
    int i;
    for(i=0;i<3;i++)
    {
		X[i][0] = A[i][0]*B[0][0] + A[i][1]*B[1][0];
    }
    return; 
}

void Matrix3x1_Add(float A[3][1], float B[3][1], float X[3][1], float scaleA, float scaleB)
{
    int i;
    for(i=0;i<3;i++)
    {
		X[i][0] = scaleA*A[i][0] + scaleB*B[i][0];
    }
    return; 
}

void Matrix3x3_Add(float A[3][3], float B[3][3], float X[3][3], float scaleA, float scaleB)
{
    int i,j;
    for(i=0;i<3;i++)
    {
		for(j=0;j<3;j++)
		{
			X[i][j] = scaleA*A[i][j] + scaleB*B[i][j];
		}
    }
    return; 
}
