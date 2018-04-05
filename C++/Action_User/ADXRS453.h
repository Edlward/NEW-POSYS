/**
  ******************************************************************************
  * @file    ADXRS453.h
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
#ifndef __ADXRS453_H
#define __ADXRS453_H

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
#include "stm32f4xx_spi.h"
/* Exported class     ------------------------------------------------------- */
#endif
class deviceADXRS453:public device<oneAxis,uint16_t>
{	
	public:
		int16_t temp;
		deviceADXRS453(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):device(SPI,GPIO,Pin){};
		virtual ~deviceADXRS453()=default;
		virtual uint16_t 		rawDataRead(uint8_t address);
		virtual void 		 		rawDataWrite(uint8_t address,uint16_t value);
		virtual void 		 		init(void);
		virtual void     		updateData(void);
		//virtual oneAxis     getData(void);
};
/* Exported function     ------------------------------------------------------- */
deviceADXRS453& getADXRS453(void);
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
