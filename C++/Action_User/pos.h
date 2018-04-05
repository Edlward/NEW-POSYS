/**
  ******************************************************************************
  * @file    pos.h
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2016.10.26
  * @brief   This file contains the headers of usart.cpp
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

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"	 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void calculatePos(void);
void UpdateEncoder();

uint16_t  getEncoder(uint8_t num); 
float* getPos(void);
uint16_t readEncoder(uint8_t num);	 

void resetPos(void);
#ifdef __cplusplus
}
#include "device.h"
/* Exported functions ------------------------------------------------------- */
void encoderMagFix(threeAxis& data);
#endif


#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
