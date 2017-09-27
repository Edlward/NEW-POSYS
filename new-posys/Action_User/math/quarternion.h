#ifndef QUARTERNION_H
#define QUARTERNION_H

#include "config.h"

Quarternion QuaternionInt(Quarternion quaternion,three_axis_d data);
three_axis Quaternion_to_Euler(Quarternion quaternion);
Quarternion Euler_to_Quaternion(three_axis Rad);


#endif


