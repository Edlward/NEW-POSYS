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
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Flash_Write(uint8_t *data,uint32_t len);
void Flash_Zero(uint32_t len);
void Flash_Read(uint8_t *data,uint32_t len);
void Flash_Init(void);
uint8_t  *GetFlashArr(void);
float    *GetResultArr(void);
uint32_t *GetCountArr(void);
uint8_t  GetFlashUpdataFlag(void);
void     SetFlashUpdateFlag(uint8_t val);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
