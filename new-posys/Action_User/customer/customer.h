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

#ifdef TEST_SUMMER
#define DMA_SEND_SIZE   100
#else
#define DMA_SEND_SIZE   28
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DataSend(void);
void SetFlag(int val);
void AT_CMD_Handle(void);
void ReportHardFault(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
