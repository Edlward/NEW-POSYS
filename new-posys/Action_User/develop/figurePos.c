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
�Զ���  ��y����Ϊ��׼
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X*0.038622517085838;
	allPara.posy=convert_Y*0.038651337725656;
	a=0.3533/180*pi;��õ�������
  real=[1/cos(a) -tan(a);0 1]*[vell1';vell2'];
	real[0]=1.00001901160992*vell[0]*0.038622517085838-0.006166326400784*vell[1]*0.038651337725656;
  real[1]=vell[1]*0.038651337725656;
*/
/*
25.18 25.19�øĽ��㷨����ϵó���
�ֶ���	��x����Ϊ��׼  
��һ25.1606371025683  �ֶ�25.2220631351932
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X;
	allPara.posy=convert_Y;
	a=-0.1304/180*pi;��õ�������
  real=[1 0;-tan(a) 1/cos(a)]*[vell1';vell2'];
	real[0]=vell[0]*0.0385959339263024;
  real[1]=-0.00227591327416601*vell[0]*0.0385959339263024+1.00000258988726*vell[1]*0.038690160280225;
*/
float dataVellAll[2];
void calculatePos(void)
{
	static uint8_t flag=0;
	static double vellLast[2]={0.0,0.0};
	double  pos_temp[2]={0,0};
	
	int16_t vell[2];
	double real[2];
	
	double zangle;
	static double last_ang=0.0;
	
	if(allPara.resetFlag)
		flag=4;
	
	if(flag<=3)
	{
		allPara.data_last[0]=allPara.codeData[0];
		allPara.data_last[1]=allPara.codeData[1];
		vell[0]=0;
		vell[1]=0;
		flag++;
	}
	else
	{
		vell[0]= (allPara.codeData[0]-allPara.data_last[0]);
		vell[1]= (allPara.codeData[1]-allPara.data_last[1]);
		
		/*�޶�����ٶ�Ϊ7m/s,�����������ٶȾ���Ϊ������*/
//		if(fabs(vell[0])>900)
//		{
//			allPara.codeData[0]=allPara.data_last[0]+vellLast[0];
//			vell[0]=vellLast[0];
//		}
//		if(fabs(vell[1])>900)
//		{
//			allPara.codeData[1]=allPara.data_last[1]+vellLast[1];
//			vell[1]=vellLast[1];
//		}
//	
		vellLast[0]=vell[0];
		vellLast[1]=vell[1];
		
		allPara.data_last[0]=allPara.codeData[0];
		allPara.data_last[1]=allPara.codeData[1];
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

	
//
	#ifdef AUTOCAR	//��yΪ��׼
		real[0]=1.00001901160992*vell[0]*0.038622517085838-0.006166326400784*vell[1]*0.038651337725656;
		real[1]=vell[1]*0.038651337725656;
	#else			//��xΪ��׼
		real[0]=vell[0]*0.0385959339263024;
		real[1]=-0.00227591327416601*vell[0]*0.0385959339263024+1.00000258988726*vell[1]*0.038690160280225;
	#endif
//	
		dataVellAll[0]=real[0];
		dataVellAll[1]=real[1];
	
	#ifdef AUTOCAR	
		allPara.vellx=(sin(zangle*0.017453292519943)*real[1]+cos(zangle*0.017453292519943)*real[0]);
		allPara.velly=(cos(zangle*0.017453292519943)*real[1]-sin(zangle*0.017453292519943)*real[0]);
	#else
		allPara.vellx=(sin(zangle*0.017453292519943)*real[1]+cos(zangle*0.017453292519943)*real[0]);
		allPara.velly=(cos(zangle*0.017453292519943)*real[1]-sin(zangle*0.017453292519943)*real[0]);
	#endif
		allPara.posx+=allPara.vellx;
		allPara.posy+=allPara.velly;
	
	/*��ö�λϵͳx��y�����ϵ��ٶ�*/
	allPara.vellx=allPara.vellx*200.f;
	allPara.velly=allPara.velly*200.f;
	
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
