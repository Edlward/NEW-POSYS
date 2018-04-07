/**
  ******************************************************************************
  * @file    user.cpp
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
#include "user.h"
#include "calculateAttitude.h"
#include "pos.h"
#include "usart.h"
#include "ADXRS453.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static uint8_t cmdState=0x00;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
static void usartACCT(uint8_t data,uint16_t &count)
{
	static uint16_t cmdId=0;
	data-='0';
	while(data<0||data>9);
	
	cmdId*=10;
	cmdId+=data;
	
	
	count++;
	if(count==7)
	{
		switch(cmdId)
		{
			case 0:
				//ACCT000 : 开始计算位姿
				cmdState|=0x01;
				break;
			case 1:
				//ACCT001	： 停止计算位姿
				cmdState&=(~(0x01));
				break;
			case 2:
				//ACCT002	 :	使能九轴融合模式
				cmdState|=0x02;
				break;
			case 3:
				//ACCT003	 :	失能九轴融合模式
				cmdState&=(~(0x02));
				break;
			case 4:
				resetPos();
				resetAttitude();
				break;
			default:
				while(1);
		}
		cmdId=0;
	}
	
	if(count>=7)
		count=0;
}

void SetCmdState(int val){
  switch(val){
  case START_COMPETE:
    cmdState|=START_COMPETE;
    break;
  case ~START_COMPETE:
    cmdState&=~START_COMPETE;
    break;
	case HEATING:
    cmdState|=HEATING;
		break;
  case ~HEATING:
    cmdState&=~HEATING;
    break;
	case STATIC_FORCE:
    cmdState|=STATIC_FORCE;
		break;
  case ~STATIC_FORCE:
    cmdState&=~STATIC_FORCE;
    break;
  }
}

//static void usartACPC(uint8_t data,uint16_t &count)
//{
//}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void usartCmdInput(uint8_t data)
{
	static uint16_t count=0;
	static uint8_t cmdType[2]={'C','T'};
	switch(count)
	{
		case 0:
			count+=(data=='A');
			break;
		case 1:
			count+=(data=='C')*2;
			count-=1;
			break;
		case 2:
		case 3:
			cmdType[count-2]=data;
			count++;
			break;
		default:
			if(cmdType[0]=='C'&&cmdType[1]=='T')
			{
				usartACCT(data,count);
			}
			else
				count=0;
			break;
	}
}
uint8_t getCmdState(void)
{
	return	cmdState;
}

void dataSend(void)
{
	auto angle=getEulerAngle();
	uint64_t* pos=reinterpret_cast<uint64_t*>(getPos());
	auto wz=getADXRS453().getData();
	
	static float lastPos[2]={0,0};
	float vell[2]={0,0};
	
	vell[0]=getPos()[0]-lastPos[0];
	vell[1]=getPos()[1]-lastPos[1];
	lastPos[0]=getPos()[0];
	lastPos[1]=getPos()[1];
	
  angle.x=vell[0];
	angle.y=vell[1];
	
	USART_SendByteData(0x0D);
	USART_SendByteData(0x0A);
	
	USART_SendByteData(&(angle));
	USART_SendByteData(pos);
	USART_SendByteData(&(wz));
	
	USART_SendByteData(0x0A);
	USART_SendByteData(0x0D);
	
	
	cout<<getPos()[0]<<'\t'<<getPos()[1]<<'\t';
	cout<<endl;
	
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE ***********/
