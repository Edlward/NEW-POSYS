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

#include "config.h"
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

/* Exported function prototypes ---------------------------一轮 25.177966283089273 二轮半径25.196754764450382--------------------*/
/* Exported functions ---------------------------------------------------------*/
uint16_t data[2];
extern AllPara_t allPara;
void calculatePos(void)
{
	static uint16_t data_last[2]={0,0};
	static uint8_t flag=0;
	
	double  pos_temp[2]={0,0};
	
	int16_t vell[2];
	
	double zangle;
	static double last_ang=0.0;
	
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
	
	zangle=allPara.Result_Angle[2]; 
	/*-180到180*/
	if(zangle-last_ang>180.0)
		zangle=(zangle-360.0+last_ang)/2.0;
	if(zangle-last_ang<-180.0)
		zangle=(zangle+360.0+last_ang)/2.0;
	
	if(zangle>180.0)
		zangle-=360.0;
	else if(zangle<-180.0)
		zangle+=360.0;
	
	last_ang=allPara.Result_Angle[2];
	
	//直角坐标系和非直角坐标系的转换

	pos[0]+=-sin(-zangle*0.017453292519943)*vell[1]+cos(-zangle*0.017453292519943)*vell[0];
	pos[1]+=+cos(-zangle*0.017453292519943)*vell[1]+sin(-zangle*0.017453292519943)*vell[0];
	
	pos_temp[0]=pos[0];
	pos_temp[1]=pos[1];
	
// 1/cos(a/2)
	pos_temp[0]=pos[0]*1.000004752846005;
  pos_temp[1]=pos[1]*1.000004752846005;
	
	float convert_X;
	float convert_Y;
	
	convert_X=pos_temp[0]*0.707106781186548+pos_temp[1]*0.707106781186548;
	convert_Y=pos_temp[0]*0.707106781186548-pos_temp[1]*0.707106781186548;
	
	//1/4096*wheelR*2*pi
	posx=convert_X*0.038622517085838;
	posy=convert_Y*0.038651337725656;
	
	USART_OUT_F(last_ang);
	USART_OUT_F(posx);
	USART_OUT_F(posy);
	USART_OUT_F(zangle);
	USART_OUT_F(vell[0]);
	USART_OUT_F(vell[1]);
	USART_Enter();
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
