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

#define UPDATE_WHEEL_R1											1
#define UPDATE_WHEEL_R2											2
#define UPDATE_ANGLE_ERROR									3
#define UPDATE_CALIBRATION_FACTOR						4
#define UPDATE_GYRO_SCALE										5
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DataSend(void);
void SetFlag(int val);
void AT_CMD_Handle(void);
void ReportHardFault(void);
void DeadWhileReport(uint8_t a);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
