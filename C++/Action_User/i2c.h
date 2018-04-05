/**
  ******************************************************************************
  * @file    
  * @author  Lxy Action
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
#ifndef __I2C_H
#define __I2C_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx_i2c.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
void I2C2_Init(void);
void I2C_write_onebyte(u16 WriteAddr,u8 DataToWrite);
u8 I2C_read_onebyte(u16 ReadAddr);								 
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
