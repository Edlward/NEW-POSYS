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
/*
自动车  以y方向为基准
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X*0.038622517085838;
	allPara.posy=convert_Y*0.038651337725656;
	a=0.3533/180*pi;测得到的误差角
  real=[1/cos(a) -tan(a);0 1]*[allPara.vell1';allPara.vell2'];
	real[0]=1.00001901160992*allPara.vell[0]*0.038622517085838-0.006166326400784*allPara.vell[1]*0.038651337725656;
  real[1]=allPara.vell[1]*0.038651337725656;
*/
/*
25.18 25.19用改进算法后拟合得出的
手动车	以x方向为基准  
轮一25.1606371025683  轮二25.2220631351932
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X;
	allPara.posy=convert_Y;
	a=-0.1304/180*pi;测得到的误差角
  real=[1 0;-tan(a) 1/cos(a)]*[allPara.vell1';allPara.vell2'];
	real[0]=allPara.vell[0]*0.0385959339263024;
  real[1]=0.00227591327416601*allPara.vell[0]*0.0385959339263024+1.00000258988726*allPara.vell[1]*0.038690160280225;
*/
/*
-0.3819
轮一25.2100214096499  轮二25.1926480912484
	//1/4096*wheelR*2*pi
	allPara.posx=convert_X;
	allPara.posy=convert_Y;
	a=-0.3819/180*pi;测得到的误差角
  real=[1 0;-tan(a) 1/cos(a)]*[allPara.vell1';allPara.vell2'];
	real[0]=allPara.vell[0]*0.0386716885045886;
  real[1]=0.00666551112481855*allPara.vell[0]*0.0386716885045886+1.00002221427254*allPara.vell[1]*0.0386450381679389;
*/
void calculatePos(void)
{
	
	double real[2]={0.0,0.0};
	double delPos[2]={0.0,0.0};
	
	double zangle=0.0;
	static double last_ang=0.0;
	
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
	
	//直角坐标系和非直角坐标系的转换  一定要注意坐标系的正方向和角度正方向一样！

		

	#ifdef TESTCAR
			real[0]=allPara.vell[0]*0.0388255696403668;
			real[1]=0.00670391008416163*allPara.vell[0]*0.0388255696403668+1.00002247095274*allPara.vell[1]*0.03866266008737;
	#else
		#ifdef AUTOCAR	//以y为标准
			real[0]=1.00001901160992*allPara.vell[0]*0.038622517085838-0.006166326400784*allPara.vell[1]*0.038651337725656;
			real[1]=allPara.vell[1]*0.038651337725656;
		#else			//以x为标准
			real[0]=allPara.vell[0]*0.0385959339263024;
			real[1]=0.00227591327416601*allPara.vell[0]*0.0385959339263024+1.00000258988726*allPara.vell[1]*0.038690160280225;
		#endif
	#endif
	

		delPos[0]=(sin(zangle*0.017453292519943)*real[1]+cos(zangle*0.017453292519943)*real[0]);
		delPos[1]=(cos(zangle*0.017453292519943)*real[1]-sin(zangle*0.017453292519943)*real[0]);

		allPara.posx+=delPos[0];
		allPara.posy+=delPos[1];
	
	/*获得定位系统x，y方向上的速度*/
	allPara.vellx=delPos[0]*200.f;
	allPara.velly=delPos[1]*200.f;
	
}

void figureVell(void)
{
	static uint8_t flag=0;
	
	if(allPara.resetFlag)
		flag=21;
	
	if(flag<=20)
	{
		allPara.data_last[0]=allPara.codeData[0];
		allPara.data_last[1]=allPara.codeData[1];
		allPara.vell[0]=0;
		allPara.vell[1]=0;
		flag++;
	}
	else
	{
		allPara.vell[0]= (allPara.codeData[0]-allPara.data_last[0]);
		allPara.vell[1]= (allPara.codeData[1]-allPara.data_last[1]);
		allPara.data_last[0]=allPara.codeData[0];
		allPara.data_last[1]=allPara.codeData[1];
	}
	
	if(allPara.vell[0]>2048)
		allPara.vell[0]-=4096;
	if(allPara.vell[0]<-2048)
		allPara.vell[0]+=4096;
	
	if(allPara.vell[1]>2048)
		allPara.vell[1]-=4096;
	if(allPara.vell[1]<-2048)
		allPara.vell[1]+=4096;
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
