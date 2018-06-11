#ifndef QUARTERNION_H
#define QUARTERNION_H

#include "config.h"

void QuaternionInt(long double quaternion[4],long double data[3] );
void QuaternionInt1(long double quaternion[4],long double data[3] );
void Quaternion_to_Euler(const long double quaternion[4],long double Rad[3] );
void Euler_to_Quaternion(const long double Rad[3],long double quaternion[4]);
#endif


