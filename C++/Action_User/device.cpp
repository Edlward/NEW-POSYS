/**
  ******************************************************************************
  * @file    device.cpp
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2017.3.13
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "device.h"
#include "I3G4250D.h"
#include "ADXRS453.h"
#include "LSM303AGR.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define MAX_DEVICE  10
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static deviceBase* deviceList[MAX_DEVICE];
static uint8_t countDevice=0;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

//类的定义在头文件中 默认构造函数
deviceBase::deviceBase()
{
	
	deviceList[countDevice]=this;
	countDevice++;
	//如果设备数大于最大设备数就进入死循环
	if(countDevice>MAX_DEVICE)
		while(1);
}
void deviceBase::devicesAllInit()
{
	for (uint8_t iter = 0; iter < countDevice; iter++)
	{
		deviceList[iter]->init();
	}
}
void deviceBase::devicesAllUpdate()
{
	for (uint8_t iter = 0; iter < countDevice; iter++)
	{
		deviceList[iter]->updateData();
	}
}
threeAxis operator+(threeAxis x,threeAxis y)
{
	threeAxis re;
	re.x=x.x+y.x;
	re.y=x.y+y.y;
	re.z=x.z+y.z;
	return re;
}
threeAxis operator-(threeAxis x,threeAxis y)
{
	threeAxis re;
	re.x=x.x-y.x;
	re.y=x.y-y.y;
	re.z=x.z-y.z;
	return re;
}
threeAxis operator*(threeAxis x,float y)
{
	threeAxis re;
	re.x=x.x*y;
	re.y=x.y*y;
	re.z=x.z*y;
	return re;
}
threeAxis operator/(threeAxis x,float y)
{
	threeAxis re;
	re.x=x.x/y;
	re.y=x.y/y;
	re.z=x.z/y;
	return re;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
