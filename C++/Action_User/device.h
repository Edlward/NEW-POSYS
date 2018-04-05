/**
  ******************************************************************************
  * @file    device.h
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2017.3.13
  * @brief   This file contains the headers of device.cpp
  ******************************************************************************
  * @attention  这里一定要看
  *  	拿来控制共同行为和动态绑定的基类，类模板是该类的子类  deviceBase
	*		所有mems器件的类模板，该类为一个抽象类 	device
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
	/*璁剧疆鎴愰粯璁ゆ瀽鏋勫嚱鏁�*/
	virtual ~deviceBase()=default;
};
template<typename dataType,typename rawDataType> class device:public deviceBase
{
	protected:
		dataType val;												//mems器件的核心数据
		GPIO_TypeDef* 			csGPIOx;				//mems器件SPI片选
		uint16_t 			   		csGPIO_Pin;			//同上
		SPI_TypeDef* 				SPIx;						//mems器件挂载的SPI总线
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
