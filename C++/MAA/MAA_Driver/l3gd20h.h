/**
  ******************************************************************************
  * @file    L3GD20H.h
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
#ifndef __L3GD20H_H
#define __L3GD20H_H

#include "stdint.h"
#include "maa_config.h"


typedef struct
{
 uint8_t FIFO_len;
 uint8_t FIFO_FTH;
 uint8_t FIFO_Empty;
 uint8_t FIFO_Full;
 float   x[32];
 float   y[32];
 float   z[32];
}L3GD20H_FIFO_Data;

void  L3GD20H_Init(void);

void  L3GD_read_gyr(three_axis *data);
void  L3GD_updateW(void);

void  L3GD_readTemp(float *val);
void  L3GD_updateTemp(void);
void  L3GD20H_FIFO_Read(L3GD20H_FIFO_Data *data);


#endif //!__L3GD20H_H

