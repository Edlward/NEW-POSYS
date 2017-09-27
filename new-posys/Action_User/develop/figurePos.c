/**
  ******************************************************************************
  * @file     
  * @author  lxy
  * @version 
  * @date    
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
#include "figurePos.h"
#include "stdint.h"
#include "figureAngle.h"
#include "config.h"
#include "math.h"
#include "usart.h"
#include "spi.h"
#include "arm_math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float posx=0,posy=0;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/


static void AxisOriginConvert(double *posx,double *posy)
{
	float convert_X;
	float convert_Y;
	
	convert_X=((*posx) + (*posy))* 0.707106781186548 ;
	convert_Y=((*posy) - (*posx))* 0.707106781186548 ;
	
	(*posx)=convert_X;
	(*posy)=convert_Y;
}

/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/


void calculatePos(void)
{
	static uint32_t data_last[2]={0,0};
	static uint32_t flag=0;
	
	static double pos[2];
	double  pos_temp[2]={0,0};
	
	uint32_t data[2];
	int32_t vell[2];
	
	float zangle;
	static float last_ang=0.0f;
	
	data[0]=SPI_ReadAS5045(0);
	data[1]=SPI_ReadAS5045(1);
	
	if(flag<=5)
	{
		data_last[0]=data[0];
		data_last[1]=data[1];
		vell[0]=0;
		vell[1]=0;
		flag++;
	}
	else
	{
		vell[0]= (data[0]-data_last[0]);
		vell[1]= (data[1]-data_last[1]);
		
		data_last[0]=data[0];
		data_last[1]=data[1];
	}
	
	if(vell[0]>2048)
		vell[0]-=4096;
	if(vell[0]<-2048)
		vell[0]+=4096;
	
	if(vell[1]>2048)
		vell[1]-=4096;
	if(vell[1]<-2048)
		vell[1]+=4096;
	
	zangle=getAngle().z;
	zangle=(zangle+last_ang)/2.0f;
	last_ang=getAngle().z;
	//¶Û½Ç
	//vell[1]=(vell[1]-vell[0]*sin(-0.2274/180.0f*PI))/cos(-0.2274/180.0f*PI);
	
	pos[0]+=(-sin(zangle/180.0f*PI)*vell[1]+cos(zangle/180.0f*PI)*vell[0]);
	pos[1]+=(+cos(zangle/180.0f*PI)*vell[1]+sin(zangle/180.0f*PI)*vell[0]);
	
	pos_temp[0]=pos[0];
	pos_temp[1]=pos[1];
	
	AxisOriginConvert(&(pos_temp[0]),&(pos_temp[1]));
	
	posx=pos_temp[0]/4096.0f*R_wheel1*2.f*PI;
	posy=pos_temp[1]/4096.0f*R_wheel2*2.f*PI;
	
}

float getPosX(void)
{
	return posx;
}
float getPosY(void)
{
	return posy;
}

void setPosX(float in)
{
	posx=in*4096.f/R_wheel1;
}
void setPosY(float in)
{
	posy=in;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
