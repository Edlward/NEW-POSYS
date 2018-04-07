/**
  ******************************************************************************
  * @file    attitudeAngle.h
  * @author  Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2017.3.19
  * @brief   This file contains the headers of usart.cpp
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
#ifndef __ATTITUDEANGLE_H
#define __ATTITUDEANGLE_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/	 
/* Exported functions ------------------------------------------------------- */
#ifdef __cplusplus
}
 /* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "action_matrix.h"
/* Exported class     -------------------------------------------------------- */
action_matrix getTransMatrix(const action_matrix&);
action_matrix integral(const action_matrix& value,const action_matrix& val,float stepSize);
#endif
/* Exported function     ------------------------------------------------------- */
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
