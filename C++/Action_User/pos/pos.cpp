/**
  ******************************************************************************
  * @file    pos.cpp
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
#include "pos.h"
#include "calculateAttitude.h"
#include "arm_math.h"
#include "spi.h"
#include "usart.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

static const int16_t encoderRange=4096;

static uint16_t encoder[2];

static float pos[2]={0,0};

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void UpdateEncoder()
{
	encoder[0]=SPI_ReadAS5045(0);
	encoder[1]=SPI_ReadAS5045(1);
	cout<<encoder[0]<<'\t'<<encoder[1]<<endl;
}

uint16_t getEncoder(uint8_t num)
{
	return encoder[num];
}

void encoderMagFix(threeAxis& data)
{
	data.x-=-71.86f*cosf(encoder[0]*PI*2/encoderRange)-18.71f*sinf(encoder[0]*PI*2/encoderRange);
	data.y-=30.88f*cosf(encoder[0]*PI*2/encoderRange)+120.0f*sinf(encoder[0]*PI*2/encoderRange);
	data.z-=77.94f*cosf(encoder[0]*PI*2/encoderRange)+12.87f*sinf(encoder[0]*PI*2/encoderRange);
	
	data.x-=8.051f*cosf(encoder[1]*PI*2/encoderRange)-54.64f*sinf(encoder[1]*PI*2/encoderRange);
	data.y-=-106.4f*cosf(encoder[1]*PI*2/encoderRange)+73.45f*sinf(encoder[1]*PI*2/encoderRange);
	data.z-=-98.74f*cosf(encoder[1]*PI*2/encoderRange)-19.28f*sinf(encoder[1]*PI*2/encoderRange);
}


void calculatePos(void)
{
	double eulerAngle=getEulerAngle().z;
	static double last_ang=0.0;
	
		/*-180µ½180*/
	if(eulerAngle-last_ang>180.0)
		eulerAngle=(eulerAngle-360.0+last_ang)/2.0;
	if(eulerAngle-last_ang<-180.0)
		eulerAngle=(eulerAngle+360.0+last_ang)/2.0;
	
	if(eulerAngle>180.0)
		eulerAngle-=360.0;
	else if(eulerAngle<-180.0)
		eulerAngle+=360.0;
	
	last_ang=eulerAngle;
	
	double real[2]={0.0,0.0};
	double delPos[2]={0.0,0.0};
	
	uint16_t encoder[2]={getEncoder(0),getEncoder(1)};
	
	static uint16_t encoderLast[2]={encoder[0],encoder[1]};
	
	int32_t vell[2]={encoder[0]-encoderLast[0],encoder[1]-encoderLast[1]};
	
	encoderLast[0]=encoder[0];
	encoderLast[1]=encoder[1];
	
	vell[0]-=(vell[0]>(encoderRange/2))*encoderRange;
	vell[0]+=(vell[0]<(-(encoderRange/2)))*encoderRange;
	
	vell[1]-=(vell[1]>(encoderRange/2))*encoderRange;
	vell[1]+=(vell[1]<(-(encoderRange/2)))*encoderRange;
	
	real[0]=1.00001901160992*vell[0]*0.038622517085838-0.006166326400784*vell[1]*0.038651337725656;
	real[1]=vell[1]*0.038651337725656;
	
	delPos[0]=(sin(eulerAngle*0.017453292519943)*real[1]+cos(eulerAngle*0.017453292519943)*real[0]);
	delPos[1]=(cos(eulerAngle*0.017453292519943)*real[1]-sin(eulerAngle*0.017453292519943)*real[0]);

	pos[0]+=delPos[0];
	pos[1]+=delPos[1];
}
float* getPos(void)
{
	return pos;
}
void resetPos(void)
{
	pos[0]=0;
	pos[1]=0;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE ***********/
