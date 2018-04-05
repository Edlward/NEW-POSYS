/**
  ******************************************************************************
  * @file    fifo.h
  * @author  Lxy Zlq
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
#ifndef __FIFO_H
#define __FIFO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
 uint8_t FIFO_len;
 uint8_t FIFO_FTH;
 uint8_t FIFO_Empty;
 uint8_t FIFO_Full;
 float   x[32];
 float   y[32];
 float   z[32];
}FIFO_Data;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void FIFO_Read(uint8_t device,FIFO_Data *data);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
