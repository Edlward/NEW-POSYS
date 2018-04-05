/**
  ******************************************************************************
  * @file    ADXRS453.h
  * @author  Luo Xiaoyi and Qiao Zhijian 
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

//use template "device" 
//this is a one-Axis mems gyroscope and it returns data of uint16_t
class deviceADXRS453:public device<oneAxis,uint16_t>
{	
	public:
		//temperature
		int16_t temp;
		//assign values to variables "csGPIOx,csGPIO_Pin,SPIx" 
		deviceADXRS453(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):device(SPI,GPIO,Pin){};
		//default destructor
		virtual ~deviceADXRS453()=default;
		//read raw data from address
		virtual uint16_t 		rawDataRead(uint8_t address);
		//write raw data into address
		virtual void 		 		rawDataWrite(uint8_t address,uint16_t value);
		//init the device
		virtual void 		 		init(void);
		//update the datas
		virtual void     		updateData(void);
		//virtual oneAxis     getData(void);
};
/* Exported function     ------------------------------------------------------- */
deviceADXRS453& getADXRS453(void);
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
