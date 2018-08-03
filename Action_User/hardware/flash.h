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
#include "config.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

void Flash_Init(void);

/*
chartMode��ʽ  
������1��X��  Y��  Z�ᣩ
������2��X��  Y��  Z�ᣩ
������3��X��  Y��  Z�ᣩ

chart+��������ţ�0-(GYRO_NUMBER-1)��*AXIS_NUMBER+��ţ�0-(AXIS_NUMBER-1����
*/
typedef struct{
	/*���������ǵ�������ٶȷ���*/
	float var[GYRO_NUMBER][AXIS_NUMBER];
	
	float temperature_I;
	
}FlashData_t;


#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
