/**
  ******************************************************************************
  * @file    signalProcess.h
  * @author  Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2017.4.2
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
#ifndef __SIGNALPROCESS_H
#define __SIGNALPROCESS_H

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
float averageWithRemoveErr(float *data,uint32_t len);


	 
#ifdef __cplusplus
}
/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "device.h"
/* Exported template	--------------------------------------------------------*/
/* Exported class     --------------------------------------------------------*/
/* Exported function     -----------------------------------------------------*/

//figure average  out
inline float	meanData(float *data,uint32_t len)
{
	float re;
	arm_mean_f32(data,len,&re);
	return re;
}
threeAxis meanData(threeAxis *data,uint32_t len);
inline float	stdData(float *data,uint32_t len)
{
	float re;
	arm_std_f32(data,len,&re);
	return re;
}
threeAxis stdData(threeAxis *data,uint32_t len);

template<typename p>
void shiftLeftData(p pointer,uint32_t len,uint32_t num=1)
{
	for(uint32_t i=0;i<len-num;i++)
	{
		pointer[i]=pointer[i+num];
	}
}

/*shift data right one unit*/
template<typename p>
void shiftRightData(p pointer,uint32_t len)
{
	for(uint32_t i=0;i<len-1;i++)
	{
		pointer[len-i-1]=pointer[len-i-2];
	}
}
template<typename p>
p meanData(p* pointer,uint32_t len)
{
	p temp;
	temp=0;
	for(uint32_t i=0;i<len;i++)
	{
		temp=temp+pointer[i]/static_cast<double>(len);
	}
	return temp;
}
#endif

#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
