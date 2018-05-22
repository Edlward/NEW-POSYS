
#ifndef __ICM_H
#define __ICM_H

#include "config.h"

#define REGISTERS 							30

//#ifndef GYRO_NUMBER
//#define GYRO_NUMBER 3
//#endif
//#ifndef AXIS_NUMBER
//#define AXIS_NUMBER 3
//#endif

void MEMS_Configure(int gyroNum);

void icm_read_gyro_rate(double data[GYRO_NUMBER]);
void icm_read_accel_acc(double data[GYRO_NUMBER]);
void icm_read_temp(float *data);

void icm_update_gyro_rate(int gyroNum);
void icm_update_acc(int gyroNum);
void icm_update_temp(int gyroNum);

int CheckNan(void);

void AllParaInit(void);
#endif 


