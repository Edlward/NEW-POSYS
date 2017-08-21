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
//#define SINGLE
#define QUATER
/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "icm_20608_g.h"
/* Exported types ------------------------------------------------------------*/



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t updateAngle(void);
uint8_t adjustVDoff(three_axis *w);
float KalmanFilterZAxis(float ordata);
void WaitForUpdataVDoff(void);		
double safe_atan2(double x,double y);			
float KalmanFilterTAxis(float ordata);
three_axis getAngle(void);
void setAngle(float zAngle);
void resetAngle(void);
void DebugMode(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
