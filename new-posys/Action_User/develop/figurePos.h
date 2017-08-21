/**
  ******************************************************************************
  * @file    *.h
  * @author  Lxy Action
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

void setPosX(float in);
void setPosY(float in);

void resetPos(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
