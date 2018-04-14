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
 
/* Exported dataType ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/	 
/* Exported functions ------------------------------------------------------- */
float averageWithRemoveErr(float *data,uint32_t len);
	 
#ifdef __cplusplus
}
#endif

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "device.h"
#include "timer.h"
/* Exported template	--------------------------------------------------------*/
/* Exported class     --------------------------------------------------------*/
/* Exported function     -----------------------------------------------------*/
#define BaisTime		(3)
#define BaisNum			(BaisTime*PERIOD)

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

template<typename dataType>
class BaisHandle
{
	private:
		int index;
		dataType rateBaisArray[BaisNum];
	public:
		dataType rateBais;
		//default constructor
		BaisHandle()
		{
			ResetBais();
			rateBais=0.f;
		}
		
		//push a data and get average of rateBaisArray
		dataType GetBais(dataType data)
		{
			dataType re=0.f;
			if(index<BaisNum-1)
			{
				index++;
				rateBaisArray[index]=data;
				rateBais=0.f;
			}
			else if(index==BaisNum-1)
			{
				index++;
				rateBaisArray[index]=data;
				re=meanData(rateBaisArray,BaisNum);
			}
			else
			{
				shiftRightData(rateBaisArray,BaisNum);
				rateBaisArray[0]=data;
				re=meanData(rateBaisArray,BaisNum);
			}
			return re;
		}
		//reset index and rateBaisArray
		void ResetBais(void)
		{
			for(index=0;index<BaisNum;index++)
			{
				rateBaisArray[index]=0.f;
			}
			index=0;
		}
};
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
