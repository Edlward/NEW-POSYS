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
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define TempTable_Num  	145				//3*5个double 存储的是随温度变化的斜率 最后一个int存储的是选几个做平均数
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Flash_Write(uint8_t *data,uint32_t len);
void Flash_Zero(uint32_t len);
void Flash_Read(uint8_t *data,uint32_t len);
void Flash_Init(void);
uint8_t  *GetFlashArr(void);
void Flash_Return(void);


typedef struct{
	double    *chartWX;
	double    *chartWY;
	double    *chartWZ;
	uint8_t 	*chartMode;
	uint8_t 	*chartSelect;
	uint8_t   *scaleMode;
	float  		*minValue;
	float     *varXYZ;
}flashData_t;


#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
