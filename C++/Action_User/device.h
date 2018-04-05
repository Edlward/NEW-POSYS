/**
  ******************************************************************************
  * @file    device.h
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2017.3.13
  * @brief   This file contains the headers of device.cpp
  ******************************************************************************
  * @attention  ����һ��Ҫ��
  *  	�������ƹ�ͬ��Ϊ�Ͷ�̬�󶨵Ļ��࣬��ģ���Ǹ��������  deviceBase
	*		����mems��������ģ�壬����Ϊһ�������� 	device
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
	virtual void 		 		init(void)=0;
	virtual void     		updateData(void)=0;
	/*设置成默认析构函数*/
	virtual ~deviceBase()=default;
};
template<typename dataType,typename rawDataType> class device:public deviceBase
{
	protected:
		dataType val;												//mems�����ĺ�������
		GPIO_TypeDef* 			csGPIOx;				//mems����SPIƬѡ
		uint16_t 			   		csGPIO_Pin;			//ͬ��
		SPI_TypeDef* 				SPIx;						//mems�������ص�SPI����
	public:
		device(SPI_TypeDef* SPI,GPIO_TypeDef* GPIO,uint16_t Pin):csGPIOx(GPIO),csGPIO_Pin(Pin),SPIx(SPI){};
		virtual rawDataType rawDataRead(uint8_t address)=0;
		virtual void 		 		rawDataWrite(uint8_t address,rawDataType value)=0;
		virtual void 		 		init(void)=0;
		virtual void     		updateData(void)=0;
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
