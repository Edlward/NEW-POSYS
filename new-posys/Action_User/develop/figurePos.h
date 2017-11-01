/**
  ******************************************************************************
  * @file    *.h
  * @author  Qzj Action
  * @version 
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
#ifndef __POS_H
#define __POS_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void calculatePos(void);
float getPosX(void);
float getPosY(void);
void SetPosX(float in);
void SetPosY(float in);
void CorrectManufacturingErrors(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
