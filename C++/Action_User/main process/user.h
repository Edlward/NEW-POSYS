/**
  ******************************************************************************
  * @file    user.h
  * @author  Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2017.4.7
  * @brief   This file contains the headers of device.cpp
  ******************************************************************************
  * @attention  
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_H
#define __USER_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"	
#include "arm_math.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
	 
#define CORRECT    								0X01
#define START_COMPETE 						0X02
#define NULL_FLAG									0X04
#define HEATING										0X08
#define STATIC_FORCE							0X10
	 
#define DEVICE_IS_RUNNING			(getCmdState()&START_COMPETE)
#define MAG_IS_ENABLE					(getCmdState()&0x02)
	 
/* Exported functions ------------------------------------------------------- */
void usartCmdInput(uint8_t data);
uint8_t getCmdState(void);
void SetCmdState(int val);
void dataSend(void);
#ifdef __cplusplus
}

/* Exported Class ----------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
#endif


#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
