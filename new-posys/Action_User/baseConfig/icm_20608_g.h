#ifndef __ICM_H
#define __ICM_H

#include "config.h"

#define REGISTERS 							30

void ICM20608G_init(void);

void ICM_Write(uint8_t address,uint8_t value);

void icm_read_gyro_rate(float data[3]);
void icm_read_temp(float *data);
void icm_read_accel_acc(float data[3]);

void icm_get_gyro_data(short *data1,short *data2);
void icm_get_accel_data(short *data1,short *data2);

void icm_update_temp(int gyroNum);
void icm_update_gyro_rate(int gyroNum);
void icm_update_acc(int gyroNum);

void icm_update_AccRad(double accInit[2],float *rad);
#endif // !__ICM_H


