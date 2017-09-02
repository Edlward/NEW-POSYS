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
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float posx=0,posy=0;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/


static void AxisOriginConvert(float rel_x,float rel_y,float angle,
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
}

/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
#ifdef SINGLESYSTEM
void calculatePos(void)
{
	static uint16_t data_last[2]={0,0};
	static uint8_t flag=0;
	
	static float pos[2];
	
	uint16_t data[2];
	 int16_t vell[2];
	
	float zangle;
	static float last_ang=0.0f;
	
	data[0]=SPI_ReadAS5045(0);
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
	
	zangle=getAngle().z;
	zangle=(zangle+last_ang)/2.0f;
	last_ang=getAngle().z;
	
		pos[1]+=vell[0]*cos(zangle/180.0f*PI);
	pos[0]+=vell[0]*cos((zangle+90.0f)/180.0f*PI);
	
	posx=pos[0]/4096.0f*R_wheel*2*PI;
	posy=pos[1]/4096.0f*R_wheel*2*PI;
	
}
#else

void calculatePos(void)
{
	static uint16_t data_last[2]={0,0};
	static uint8_t flag=0;
	
	static float pos[2];
	float  pos_temp[2]={0,0};
	
	uint16_t data[2];
	int16_t vell[2];
	
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
	//钝角
	//vell[1]=(vell[1]-vell[0]*sin(-0.2274/180.0f*PI))/cos(-0.2274/180.0f*PI);
	
	pos[0]+=(-sin(zangle/180.0f*PI)*vell[1]+cos(zangle/180.0f*PI)*vell[0]);
	pos[1]+=(+cos(zangle/180.0f*PI)*vell[1]+sin(zangle/180.0f*PI)*vell[0]);
	
	pos_temp[0]=pos[0];
	pos_temp[1]=pos[1];
	
	AxisOriginConvert(0,0,45,getAngle().z,&(pos_temp[0]),&(pos_temp[1]));
	
	posx=pos_temp[0]/4096.0f*R_wheel1*2*PI;
	posy=pos_temp[1]/4096.0f*R_wheel2*2*PI;
	
	#ifdef DEBUG_ENABLE
	USART_OUT_F(posx);
	USART_OUT_F(posy);
	USART_OUT(USART1,(uint8_t*)"\r\n");
	#endif
}
#endif
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
