#ifndef LEASTSQUARE_H
#define LEASTSQUARE_H

#include "config.h"

#define MaxCoeff			 2

void SquareFitting(float data[(int)((TempTable_max - TempTable_min)/TempStep)]);
float FitResult(float tem);
#endif


