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
#ifndef __GYRO_H
#define __GYRO_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"	

#ifdef __cplusplus
}

#include "device.h"	 
/* Exported functions ------------------------------------------------------- */
threeAxis getEulerAngle(void);


#endif


#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/


