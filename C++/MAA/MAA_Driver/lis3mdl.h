/**
  ******************************************************************************
  * @file    LIS3MDL.h
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
#ifndef __LIS3MDL_H
#define __LIS3MDL_H

#include "stdint.h"
#include "maa_config.h"

void LIS3MDL_Init(void);
void LIS3MDL_UpdateMag(void);
void LIS_readMag(three_axis *val);
#endif // !__LIS3MDL_H

