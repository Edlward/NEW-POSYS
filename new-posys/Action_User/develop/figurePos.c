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
static float pos[2];
 float set_x,set_y,set_angle;	 
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
uint16_t data[2];
void calculatePos(void)
{
	static uint16_t data_last[2]={0,0};
	static uint8_t flag=0;
	extern int set_all;
	extern int set_d;
	
	float  pos_temp[2]={0,0};
	
	int16_t vell[2];
	
	float zangle;
	static float last_ang=0.0f;
	
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
	
	
	//直角坐标系和非直角坐标系的转换
//	pos[1]+=vell[1]*cos((zangle+45)/180.0f*PI)+vell[0]*cos((45-zangle)/180.0f*PI);
//	pos[0]+=vell[0]*cos((zangle+45)/180.0f*PI)-vell[1]*cos((45-zangle)/180.0f*PI);
	
	pos[0]+=(-sin(zangle/180.0f*PI)*vell[1]+cos(zangle/180.0f*PI)*vell[0]);
	pos[1]+=(+cos(zangle/180.0f*PI)*vell[1]+sin(zangle/180.0f*PI)*vell[0]);
	
	pos_temp[0]=pos[0];
	pos_temp[1]=pos[1];
	
	//钝角
//		pos_temp[0]=pos[0]*cos(0.7/180.0f*PI)-pos[1]*sin(0.7/180.0f*PI);
//	pos_temp[1]=pos[1]*cos(0.7/180.0f*PI)-pos[0]*sin(0.7/180.0f*PI);
//	
	
//	
//	//锐角
//			pos_temp[0]=pos[0]*cos(0.2/180.0f*PI)+pos[1]*sin(0.2/180.0f*PI);
//    	pos_temp[1]=pos[1]*cos(0.2/180.0f*PI)+pos[0]*sin(0.2/180.0f*PI);
//	
	
	AxisOriginConvert(0,0,45,getAngle().z,&(pos_temp[0]),&(pos_temp[1]));
	
	posx=(double)pos_temp[0]/4096.0*R_wheel1*2.0*(double)PI;
	posy=(double)pos_temp[1]/4096.0*R_wheel2*2.0*(double)PI;
	
//	if(set_all==1)
//	{		
//		setAngle(set_angle);
//		setPosX_Y(set_x,set_y);
//		set_all=0;
//	}
//		if(set_d==1)
//	{		
//		setPosX_Y(set_x,set_y);
//		set_d=0;
//	}
	#ifdef TEST_SUMMER
//		USART_OUT_F(posx);
//		USART_OUT_F(posy);
//		USART_Enter();
	#endif
}
float getPosX(void)
{
	return posx;
}
float getPosY(void)
{
	return posy;
}

void SetPosX(float in)
{
  posx=in;
}
void SetPosY(float in)
{
  posy=in;
}

//void CorrectManufacturingErrors(void){
//	  static uint32_t data_last[2]={0,0};
//  static uint32_t flag=0;
//  
//  static double pos[2];
//  double  pos_temp[2]={0,0};
//  
//  uint32_t data[2];
//  int32_t vell[2];
//  
//  float zangle;
//  static float last_ang=0.0f;
//  
//  data[0]=SPI_ReadAS5045(0);
//  data[1]=SPI_ReadAS5045(1);
//  
//  if(flag<=5)
//  {
//    data_last[0]=data[0];
//    data_last[1]=data[1];
//    vell[0]=0;
//    vell[1]=0;
//    flag++;
//  }
//  else
//  {
//    vell[0]= (data[0]-data_last[0]);
//    vell[1]= (data[1]-data_last[1]);
//    
//    data_last[0]=data[0];
//    data_last[1]=data[1];
//  }
//  
//  if(vell[0]>2048)
//    vell[0]-=4096;
//  if(vell[0]<-2048)
//    vell[0]+=4096;
//  
//  if(vell[1]>2048)
//    vell[1]-=4096;
//  if(vell[1]<-2048)
//    vell[1]+=4096;
//  
//	USART_OUT(USART1,"%d\t",vell[0]);
//	USART_OUT(USART1,"%d\r\n",vell[1]);

//}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
