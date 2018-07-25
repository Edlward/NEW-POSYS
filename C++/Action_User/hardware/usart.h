/**
  ******************************************************************************
  * @file    usart.h
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
#ifndef __USART_H
#define __USART_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#define DEBUG	 
//#define HEX_SEND	 
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
	 
#ifdef DEBUG
	#define cout (getUsartOut())
	#define endl ("\r\n")	 	 
#endif	 

/* Exported functions ------------------------------------------------------- */
void USART1_Init(void);
void USART6DMAInit(uint32_t BaudRate);
void USART_SendDataToDMA_USART6(uint8_t data);
void USART1_IRQHandler(void);	 
void USART_SendDataToDMA_USART1(uint8_t data);
#ifdef __cplusplus
}
/* Exported functions ------------------------------------------------------- */
  #ifdef DEBUG 
	class _out_stream
	{
		public:
			const _out_stream& operator<<(const int32_t value) const;
			inline const _out_stream& operator<<(const int16_t value) const{	return this->operator<<(static_cast<int32_t>(value));	}
			inline const _out_stream& operator<<(const int8_t value) const{	return this->operator<<(static_cast<int32_t>(value));	}
			
			const _out_stream& operator<<(const uint32_t value) const;
			inline const _out_stream& operator<<(const uint16_t value) const{	return this->operator<<(static_cast<uint32_t>(value));	}
			
			const _out_stream& operator<<(const float value) const;
			inline const _out_stream& operator<<(const double value) const{	return this->operator<<(static_cast<float>(value));	}
			
			const _out_stream& operator<<(const char value) const;
			const _out_stream& operator<<(const char* value) const;
	};
	
  _out_stream& getUsartOut(void);

	template<typename val>
	void USART_SendByteData(val *data)
	{
		#ifdef HEX_SEND
		uint8_t len;
		len=sizeof(val);
		
		uint8_t *p=reinterpret_cast<uint8_t*>(data);
		
		for(uint8_t i=0;i<len;i++)
		{
			USART_SendDataToDMA_USART1(p[i]);
		}
		#endif
	}
	inline void USART_SendByteData(uint8_t data)
	{
		#ifdef HEX_SEND
			USART_SendDataToDMA_USART1(data);
		#endif
	}
	
	
	
  #endif
#endif


#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
