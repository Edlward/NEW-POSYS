/**
  ******************************************************************************
  * @file    MAA.h
  * @author  Lxy Zlq Action
  * @version Version 1.0
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAA_H
#define __MAA_H

/* Includes ------------------------------------------------------------------*/
#include "maa_config.h"
#include "lsm303c.h"
#include "lsm6ds33.h"
#include "lis3mdl.h"
#include "l3gd20h.h"
#include "icm_20608_g.h"
/* Exported types ------------------------------------------------------------*/



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t updateAngle(void);
uint8_t adjustVDoff(three_axis *w);
float KalmanFilterZAxis(float ordata);
float KalmanFilterXAxis(float ordata);
float KalmanFilterYAxis(float ordata);	
void WaitForUpdataVDoff(void);		
double safe_atan2(double x,double y);			

three_axis getAngle(void);
void setAngle(float zAngle);
void resetAngle(void);

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
