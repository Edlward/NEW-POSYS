/**
  ******************************************************************************
  * @file    device.h
  * @author  Luo Xiaoyi and Qiao Zhijian 
  * @version V1.0
  * @date    2017.3.13
  * @brief   This file contains the headers of device.cpp
  ******************************************************************************

  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICE_H
#define __DEVICE_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"	
#include "stm32f4xx_gpio.h"	 
#include "arm_math.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/	 
/* Exported functions ------------------------------------------------------- */
#ifdef __cplusplus
}

/* Exported Class ----------------------------------------------------------- */
class deviceBase
{
private:
public:
	deviceBase();
	static 	void devicesAllInit(); 
	static 	void devicesAllUpdate(); 
	/*pure virtual function that can't make the definite and significative realizetion*/
	virtual void 		 		init(void)=0;
	virtual void     		updateData(void)=0;
	/*set default constructor*/
	virtual ~deviceBase()=default;
};

//this a template class, which dataType indicates the form(three axises, one axis...) of data and rawDataType indicates data type(char,int...) 
template<typename dataType,typename rawDataType> 
class device:public deviceBase
{
	protected:
		dataType val;												//mems core datas
		GPIO_TypeDef* 			csGPIOx;				//mems CS
		uint16_t 			   		csGPIO_Pin;			//idem
		SPI_TypeDef* 				SPIx;						//mems SPI
	public:
		//Initialize the member 			variables csGPIOx=GPIO,csGPIO_Pin=Pin,SPIx=SPI
		device(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):csGPIOx(GPIO),csGPIO_Pin(Pin),SPIx(SPI){};
		//read raw data from address
		virtual rawDataType rawDataRead(uint8_t address)=0;
		//write raw data into address
		virtual void 		 		rawDataWrite(uint8_t address,rawDataType value)=0;
		//init the device
		virtual void 		 		init(void)=0;
		//update the datas
		virtual void     		updateData(void)=0;
		//get the data
		virtual dataType    getData(void){ return  val;}
		virtual ~device()=default;
};
struct threeAxis
{
	float z;
	float x;
	float y;
public:
	inline void operator=(const threeAxis& val)
	{
		x=val.x;
		y=val.y;
		z=val.z;
	}
	inline void operator=(float val)
	{
		x=val;
		y=val;
		z=val;
	}
};	 
struct twoAxis
{
	float x;
	float y;
public:
	inline void operator=(const threeAxis& val)
	{
		x=val.x;
		y=val.y;
	}
	inline void operator=(float val)
	{
		x=val;
		y=val;
	}
};
typedef float oneAxis;
/* Exported functions ------------------------------------------------------- */
threeAxis operator+(threeAxis,threeAxis);
threeAxis operator-(threeAxis,threeAxis);
threeAxis operator*(threeAxis,float);
threeAxis operator/(threeAxis,float);
inline float abs(threeAxis x)
{
	return sqrt(x.x*x.x+x.y*x.y+x.z*x.z);
}
#endif


#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
