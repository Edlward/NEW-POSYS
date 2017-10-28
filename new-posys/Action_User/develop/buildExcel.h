/**
  ******************************************************************************
  * @file    vdoff.h
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
#ifndef __VDOFF_H
#define __VDOFF_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void UpdataExcel(void);
int UpdateVDoffTable(void);
void TempTablePrintf(void);
void Hex_To_Str(uint8_t * pHex,char * s,float num);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
