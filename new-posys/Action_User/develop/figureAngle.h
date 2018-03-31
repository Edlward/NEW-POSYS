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
/* Exported types ------------------------------------------------------------*/



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

int RoughHandle(void);

void TemporaryHandle(void);

void updateAngle(void);

double safe_atan2(double x,double y);			

void setAngle(float zAngle);

float safe_asin(float v);

void SetAngle(float angle);

void driftCoffecientInit(void);

void JudgeStatic(void);

uint8_t UpdateBais(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
