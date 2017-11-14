#ifndef QUARTERNION_H
#define QUARTERNION_H

#include "config.h"

void QuaternionInt(double quaternion[4],float data[3] );
void QuaternionInt1(double quaternion[4],float data[3] );
void Quaternion_to_Euler(const double quaternion[4],float Rad[3] );
void Euler_to_Quaternion(const float Rad[3],double quaternion[4]);
#endif


