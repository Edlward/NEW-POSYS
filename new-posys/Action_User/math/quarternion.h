#ifndef QUARTERNION_H
#define QUARTERNION_H

#include "config.h"

void QuaternionInt(double quaternion[4],double data[3] );
void QuaternionInt1(double quaternion[4],double data[3] );
void Quaternion_to_Euler(const double quaternion[4],double Rad[3] );
void Euler_to_Quaternion(const double Rad[3],double quaternion[4]);
#endif


