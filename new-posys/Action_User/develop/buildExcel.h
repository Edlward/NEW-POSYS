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
void WaitForUpdataVDoff(void);
void TempTablePrintf(float *result);
float *GetVdoff_icmArr(void);
float **GetVdoff_icmErrArr(void);
uint32_t *GetCountnum_icmArr(void);
void UpdateVDoffTable(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
