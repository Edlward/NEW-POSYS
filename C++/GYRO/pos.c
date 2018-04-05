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
#include "pos.h"
#include "stdint.h"
#include "motion_attitude_algorithm.h"
#include "maa_config.h"
#include "math.h"
#include "usart.h"
#include "spi.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float posx=0,posy=0;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/*static void AxisOriginConvert(float rel_x,float rel_y,float angle,
	                            float act_angle,float *posx,float *posy)
{
	float convert_X;
	float convert_Y;
  float rel_angle;
  float rel_sum;  
	
	rel_sum=sqrt(rel_x*rel_x+rel_y*rel_y);
	rel_angle=atan2(rel_y,rel_x);
	
	angle=angle/180.0f*PI;
	act_angle=act_angle/180.0f*PI;
	
	convert_X=+(*posx)*cos(angle)+(*posy)*sin(angle)+rel_sum*cos(act_angle+rel_angle)-rel_sum*cos(rel_angle);
	convert_Y=-(*posx)*sin(angle)+(*posy)*cos(angle)+rel_sum*sin(act_angle+rel_angle)-rel_sum*sin(rel_angle);
	
	(*posx)=convert_X;
	(*posy)=convert_Y;
}*/

/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
	uint16_t data[2];
	int route_summer=0;
void calculatePos(void)
{
	static uint16_t data_last[2]={0,0};
	static uint8_t flag=0;
	
	static float pos[2];
	
	 int16_t vell[2];
	
	float zangle;
	static float last_ang=0.0f;
	
	data[0]=SPI_ReadAS5045(1);
	/*消除空程*/
	if(flag<=5)
	{
		data_last[0]=data[0];
		vell[0]=0;
		flag++;
	}
	else
	{
		vell[0]=-(data[0]-data_last[0]);
		
		data_last[0]=data[0];
	}
	/*起点和终点相连时*/
	if(vell[0]>2048)
		vell[0]-=4096;
	if(vell[0]<-2048)
		vell[0]+=4096;
	
	route_summer+=vell[0];
	
//	zangle=getAngle().z;
//	zangle=(zangle+last_ang)/2.0f;
//	last_ang=getAngle().z;
//	
//	
//	pos[1]+=vell[0]*cos(zangle/180.0f*PI);/*/4096.0f*R_wheel*2*PI;*/
//	pos[0]+=vell[0]*cos((zangle+90.0f)/180.0f*PI);/*/4096.0f*R_wheel*2*PI;*/
////	
//	posx=pos[0]/4096.0f*R_wheel*2*PI;
//	posy=pos[1]/4096.0f*R_wheel*2*PI;
	
	
	
}
float getPosX(void)
{
	return posx;
}
float getPosY(void)
{
	return posy;
}
void resetPos(void)
{
	posx=0;
	posy=0;
}

void setPosX(float in)
{
	posx=in;
}
void setPosY(float in)
{
	posy=in;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
