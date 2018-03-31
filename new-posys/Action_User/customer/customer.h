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
#ifndef __CUSTOMER_H
#define __CUSTOMER_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DataSend(void);
void SetFlag(int val);
void AT_CMD_Handle(void);
void debugsend(float a,float b,float c,float d,float e,float f);
void debugsend2(float a,float b,float c,float d,float e);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
