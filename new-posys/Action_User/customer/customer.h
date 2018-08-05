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
void ReportHardFault(void);
void DeadWhileReport(uint8_t a);
int32_t getEncoderSum(int num);
double getDirectLine(float wheel1,float wheel2,float errorAngle);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
