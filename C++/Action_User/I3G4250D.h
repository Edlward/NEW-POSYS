/**
  ******************************************************************************
  * @file    I3G4250D.h
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
#ifndef __I3G4250D_H
#define __I3G4250D_H

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
/* Exported functions ------------------------------------------------------- */

#endif

#include "device.h"

class deviceI3G4250D:public device<threeAxis,uint8_t>
{	
	private:
		void multiRead(uint8_t address,uint8_t *data,uint32_t len);
	public:
		int16_t temp;
		deviceI3G4250D(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):device(SPI,GPIO,Pin){};
		virtual ~deviceI3G4250D()=default;
		virtual uint8_t 		rawDataRead(uint8_t address);
		virtual void 		 		rawDataWrite(uint8_t address,uint8_t value);
		virtual void 		 		init(void);
		virtual void     		updateData(void);
		//virtual threeAxis   getData(void);
};
deviceI3G4250D& getI3G4250D(void);
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
