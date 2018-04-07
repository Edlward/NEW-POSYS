/**
  ******************************************************************************
  * @file    LSM6DS33.h
  * @author  zlq lxy
  * @version 
  * @date    2016-02-21
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
#ifndef __LSM6DS33_H
#define __LSM6DS33_H

#include "stdint.h"
#include "maa_config.h"

typedef struct
{
 uint16_t Len;
 uint8_t Empty;
 uint8_t Full;
 uint8_t OR;
 uint8_t FTH;
 float temp;
 float wx[16];
 float wy[16];
 float wz[16];
}LSM6D_FIFO;


void LSM6DS33_Init(void);
void LSM6DS33_UpdateTemp(void);
void LSM6DS33_ReadTemp(float *temp);
void LSM6DS33_UpdateW(void);
void LSM6DS33_UpdateACC(void);
void LSM6D_ReadGyr(three_axis *gyr);
void LSM6D_ReadAcc(three_axis *acc);
void LSM6D_VDoff_Init(void);
BOOL LSM6D_FIFO_Read(LSM6D_FIFO *data);
void LSM6D_ReadAccRad(three_axis *rad);
#endif // !__LSM6DS33_H

