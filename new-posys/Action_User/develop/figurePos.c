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
/*
�Զ���  
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X*0.038622517085838;
	allPara.posy=convert_Y*0.038651337725656;
	a=0.3533/2/180*pi;��õ�������
  real=[ cos(a)/(2*cos(a)^2 - 1), sin(a)/(2*sin(a)^2 - 1);sin(a)/(2*sin(a)^2 - 1), cos(a)/(2*cos(a)^2 - 1)]*[vell1';vell2'];
	real[0]=1.00001425869615*vell[0]-0.0030831778541919*vell[1];
  real[1]=-0.0030831778541919*vell[0]+1.00001425869615*vell[1];
*/
/*
25.18 25.19�øĽ��㷨����ϵó���
�ֶ���  ��һ25.1606371025683  �ֶ�25.2220631351932
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X;
	allPara.posy=convert_Y;
	a=-0.1304/2/180*pi;��õ�������
  real=[ cos(a)/(2*cos(a)^2 - 1), sin(a)/(2*sin(a)^2 - 1);sin(a)/(2*sin(a)^2 - 1), cos(a)/(2*cos(a)^2 - 1)]*[vell1';vell2'];
	real[0]=1.00000776970874*vell[0]*0.0385959339263024+0.00227593095734927*vell[1]*0.038690160280225;
  real[1]=0.00227593095734927*vell[0]*0.0385959339263024+1.00000776970874*vell[1]*0.038690160280225;
*/
void calculatePos(void)
{
	static uint16_t data_last[2]={0,0};
	static uint8_t flag=0;
	
	double  pos_temp[2]={0,0};
	
	int16_t vell[2];
	double real[2];
	
	double zangle;
	static double last_ang=0.0;
	
	if(flag<=5)
	{
		data_last[0]=allPara.codeData[0];
		data_last[1]=allPara.codeData[1];
		vell[0]=0;
		vell[1]=0;
		flag++;
	}
	else
	{
		vell[0]= (allPara.codeData[0]-data_last[0]);
		vell[1]= (allPara.codeData[1]-data_last[1]);
		
		data_last[0]=allPara.codeData[0];
		data_last[1]=allPara.codeData[1];
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
	/*-180��180*/
	if(zangle-last_ang>180.0)
		zangle=(zangle-360.0+last_ang)/2.0;
	if(zangle-last_ang<-180.0)
		zangle=(zangle+360.0+last_ang)/2.0;
	
	if(zangle>180.0)
		zangle-=360.0;
	else if(zangle<-180.0)
		zangle+=360.0;
	
	last_ang=allPara.Result_Angle[2];
	
	//ֱ������ϵ�ͷ�ֱ������ϵ��ת��  һ��Ҫע������ϵ��������ͽǶ�������һ����
	allPara.Result_Angle[0]=vell[0];
	allPara.Result_Angle[1]=vell[1];
//	
		real[0]=1.00000776970874*vell[0]+0.00227593095734927*vell[1];
		real[1]=0.00227593095734927*vell[0]+1.00000776970874*vell[1];
//	
	pos[0]+=sin(zangle*0.017453292519943)*vell[1]+cos(zangle*0.017453292519943)*vell[0];
	pos[1]+=cos(zangle*0.017453292519943)*vell[1]-sin(zangle*0.017453292519943)*vell[0];
	
	pos_temp[0]=pos[0];
	pos_temp[1]=pos[1];
	
	double convert_X;
	double convert_Y;
	
	convert_X=pos_temp[0]*0.707106781186548+pos_temp[1]*0.707106781186548;
	convert_Y=pos_temp[0]*0.707106781186548-pos_temp[1]*0.707106781186548;
	
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X*0.0385959339263024;
	allPara.posy=convert_Y*0.038690160280225;
	
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
