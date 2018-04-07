/**
  ******************************************************************************
  * @file    calculateAttitude.h
  * @author  Tian Chang & Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2016.10.26
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
#ifndef __CALCULATEATTITUDE_H
#define __CALCULATEATTITUDE_H

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
void initAHRS(void);
void updateAHRS(void);
void resetAttitude(void);
#ifdef __cplusplus
}
#include "device.h"	 
/* Exported functions ------------------------------------------------------- */
threeAxis getEulerAngle(void);
#endif


#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
