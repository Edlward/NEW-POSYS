#ifndef __ICM_H
#define __ICM_H

#include "config.h"

#define REGISTERS 							30

void ICM20608G_init(void);
void I3G4250D_init(void);

BOOL icm_check_whoami(void);
void icm_get_gyro_data(short *data1,short *data2);
void icm_get_accel_data(short *data1,short *data2);

void icm_update_temp(void);
void icm_read_temp(float *data);

void icm_update_gyro_rate(void);
void icm_update_acc(void);

void icm_read_gyro_rate(gyro_t *data);
void icm_read_accel_acc(gyro_t *val);

void icm_get_temp(float *temp);

void icm_set_gyro_bias(long *gyro_bias);
void icm_set_accel_bias(const long *accel_bias);
void icm_read_accel_bias(long *accel_bias);
void icm_read_gyro_bias(long *gyro_bias);

void icm_fifo_enable(void);
void icm_read_fifo(short *gyro);
void icm_update_AccRad(three_axis *val);
#endif // !__ICM_H


