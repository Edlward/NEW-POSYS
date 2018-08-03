#ifndef _SELF_MATH_H
#define _SELF_MATH_H



double safe_atan2(double x,double y);			

void setAngle(float zAngle);

float safe_asin(float v);

int CalculateRealCrAndMean(float stdCr[AXIS_NUMBER],float mean[AXIS_NUMBER]);

void JudgeStatic(void);

float FilterVell(float newValue);

#endif


