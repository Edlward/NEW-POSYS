/**
  ******************************************************************************
  * @file    device.cpp
  * @author  Luo Xiaoyi and Qiao Zhijian 
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
#include "ICM20602.h"
#include <typeinfo>
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define MAX_DEVICE  10
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
//save the pointers of subclasses
static deviceBase* deviceList[MAX_DEVICE];
static uint8_t countDevice=0;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

//the definition of class "deviceBase" is written at head file "device.h"
//default constructor
//this default constructor will be called at every initialization of its subclass, 
//then we can see how many devices are created and deposit the pointers of these subclasses in the array deviceList.
deviceBase::deviceBase()
{
	deviceList[countDevice]=this;
	countDevice++;
	//if the number of devices is greater than MAX_DEVICE, then procedure will run into dead while.
	if(countDevice>MAX_DEVICE)
		while(1);
}

//
void deviceBase::devicesAllInit()
{
	//iterator
	
	for (uint8_t iter = 0; iter < countDevice; iter++)
	{
		deviceList[iter]->init();
	}
}
void deviceBase::devicesAllUpdate()
{
	for (uint8_t iter = 0; iter < countDevice; iter++)
	{
		deviceList[iter]->UpdateData();
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
