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
extern AllPara_t allPara;
static double pos[2];
float set_x,set_y,set_angle;	 
uint16_t data[2];
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
	
	double convert_X;
	double convert_Y;
	
	convert_X=pos_temp[0]*0.707106781186548+pos_temp[1]*0.707106781186548;
	convert_Y=pos_temp[0]*0.707106781186548-pos_temp[1]*0.707106781186548;
	
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X*0.038622517085838;
	allPara.posy=convert_Y*0.038651337725656;
	
}

void SetPosX(double in)
{
  allPara.posx=in;
}
void SetPosY(double in)
{
  allPara.posy=in;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
