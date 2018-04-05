/**
  ******************************************************************************
  * @file    LSM303AGR.h
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2017.3.13
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
#ifndef __LSM303AGR_H
#define __LSM303AGR_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"	 
#include "spi.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/	 
/* Exported functions ------------------------------------------------------- */
#ifdef __cplusplus
}
/* Exported functions ------------------------------------------------------- */
#endif
#include "device.h"
class deviceLSM303AGR_Acc:public device<threeAxis,uint8_t>
{	
	public:
		deviceLSM303AGR_Acc(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):device(SPI,GPIO,Pin){};
		virtual ~deviceLSM303AGR_Acc()=default;
		virtual uint8_t 		rawDataRead(uint8_t address);
		virtual void 		 		rawDataWrite(uint8_t address,uint8_t val);
		virtual void 		 		init(void);
		virtual void     		updateData(void);
};
class deviceLSM303AGR_Mag:public deviceLSM303AGR_Acc
{	
	public:
		deviceLSM303AGR_Mag(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):deviceLSM303AGR_Acc(SPI,GPIO,Pin){};
		virtual ~deviceLSM303AGR_Mag()=default;
		virtual void 		 		init(void);
		virtual void     		updateData(void);
};
deviceLSM303AGR_Acc& getLSM303AGR_Acc(void);
deviceLSM303AGR_Mag& getLSM303AGR_Mag(void);
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
