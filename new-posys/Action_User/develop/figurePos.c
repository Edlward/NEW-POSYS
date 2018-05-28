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
#include "odom.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
extern AllPara_t allPara; 
/*
自动车  以y方向为基准
	//1/4096*wheelR*2*pi
	allPara.sDta.posx=convert_X*0.0385305294301115;
	allPara.sDta.posy=convert_Y*0.0383709954281714;
	a=1.371/180*pi;测得到的误差角
  real=[1/cos(a) -tan(a);0 1]*[allPara.sDta.vell1';allPara.sDta.vell2'];
			real[0]=1.00028635401126*allPara.vell[0]*0.0387451841901678-0.0239330320090246*allPara.vell[1]*0.0387451841901678;
			real[1]=allPara.vell[1]*0.0387451841901678;
*/
/*
25.18 25.19用改进算法后拟合得出的
手动车	以x方向为基准  
轮一25.1606371025683  轮二25.2220631351932
	//1/4096*wheelR*2*pi
	allPara.sDta.posx=convert_X;
	allPara.sDta.posy=convert_Y;
	a=-0.1304/180*pi;测得到的误差角
  real=[1 0;-tan(a) 1/cos(a)]*[allPara.sDta.vell1';allPara.sDta.vell2'];
	real[0]=allPara.vell[0]*0.0385959339263024;
  real[1]=0.00227591327416601*allPara.vell[0]*0.0385959339263024+1.00000258988726*allPara.vell[1]*0.038690160280225;
*/
/*
-0.3819
轮一25.2100214096499  轮二25.1926480912484
	//1/4096*wheelR*2*pi
	allPara.sDta.posx=convert_X;
	allPara.sDta.posy=convert_Y;
	a=-0.3819/180*pi;测得到的误差角
  real=[1 0;-tan(a) 1/cos(a)]*[allPara.sDta.vell1';allPara.sDta.vell2'];
	real[0]=allPara.vell[0]*0.0386716885045886;
  real[1]=0.00666551112481855*allPara.vell[0]*0.0386716885045886+1.00002221427254*allPara.vell[1]*0.0386450381679389;
*/
void calculatePos(void)
{
	
	double real[2]={0.0,0.0};
	double delPos[2]={0.0,0.0};
	
	double zangle=0.0;
	static double last_ang=0.0;

	if(allPara.resetFlag)
		last_ang=allPara.sDta.Result_Angle[2];
	
	zangle=allPara.sDta.Result_Angle[2]; 
	/*-180到180*/
	if(zangle-last_ang>180.0)
		zangle=(zangle-360.0+last_ang)/2.0;
	if(zangle-last_ang<-180.0)
		zangle=(zangle+360.0+last_ang)/2.0;
	
	if(zangle>180.0)
		zangle-=360.0;
	else if(zangle<-180.0)
		zangle+=360.0;
	
	last_ang=allPara.sDta.Result_Angle[2];
	
	//直角坐标系和非直角坐标系的转换  一定要注意坐标系的正方向和角度正方向一样！

	#ifdef AUTOCAR	//以y为标准
		real[0]=allPara.sDta.vellF[0];
		real[1]=1.00008334242607*(double)allPara.sDta.vellF[1]-0.0129109177866286*(double)allPara.sDta.vellF[0];
	#else			//以x为标准
		real[0]=allPara.sDta.vellF[0];
		real[1]=allPara.sDta.vellF[1];
	#endif

	delPos[0]=(sin(zangle*0.017453292519943)*real[1]+cos(zangle*0.017453292519943)*real[0]);
	delPos[1]=(cos(zangle*0.017453292519943)*real[1]-sin(zangle*0.017453292519943)*real[0]);

	allPara.sDta.posx+=delPos[0];
	allPara.sDta.posy+=delPos[1];
	
	/*获得定位系统x，y方向上的速度*/
	allPara.sDta.vellx=delPos[0]*200.f;
	allPara.sDta.velly=delPos[1]*200.f;
	
}

void figureVell(void)
{
	static uint8_t flag=0;
	
	#ifdef TLE5012_USED
		//第一次判断静止时可以不用判断角速度
		allPara.sDta.codeData[0]=TLE5012ReadAbsPos_A();
		allPara.sDta.codeData[1]=TLE5012ReadAbsPos_B();
	#else
		//第一次判断静止时可以不用判断角速度
		allPara.sDta.codeData[0]=SPI_ReadAS5045(0);
		allPara.sDta.codeData[1]=SPI_ReadAS5045(1);
	#endif
	
	if(allPara.resetFlag)
		flag=21;
	
	if(flag<=20)
	{
		allPara.sDta.data_last[0]=allPara.sDta.codeData[0];
		allPara.sDta.data_last[1]=allPara.sDta.codeData[1];
		allPara.vell[0]=0;
		allPara.vell[1]=0;
		flag++;
	}
	else
	{
		allPara.vell[0]= (allPara.sDta.codeData[0]-allPara.sDta.data_last[0]);
		allPara.vell[1]= (allPara.sDta.codeData[1]-allPara.sDta.data_last[1]);
		allPara.sDta.data_last[0]=allPara.sDta.codeData[0];
		allPara.sDta.data_last[1]=allPara.sDta.codeData[1];
	}
	
	#ifdef TLE5012_USED
		if(allPara.vell[0]>16384)
			allPara.vell[0]-=32768;
		if(allPara.vell[0]<-16384)
			allPara.vell[0]+=32768;
		
		if(allPara.vell[1]>16384)
			allPara.vell[1]-=32768;
		if(allPara.vell[1]<-16384)
			allPara.vell[1]+=32768;
		
		allPara.sDta.vellF[0]=allPara.vell[0]*0.310270529087862;
		allPara.sDta.vellF[1]=allPara.vell[1]*0.309998405357997;
	#else
		if(allPara.vell[0]>2048)
			allPara.vell[0]-=4096;
		if(allPara.vell[0]<-2048)
			allPara.vell[0]+=4096;
		
		if(allPara.vell[1]>2048)
			allPara.vell[1]-=4096;
		if(allPara.vell[1]<-2048)
			allPara.vell[1]+=4096;
		
		allPara.sDta.vellF[0]=allPara.vell[0]*0.0387838161359827;
		allPara.sDta.vellF[1]=allPara.vell[1]*0.0387498006697496;
	#endif
	
	allPara.vellSum[0]+=allPara.vell[0];
	allPara.vellSum[1]+=allPara.vell[1];
}

void SetPosX(double in)
{
  allPara.sDta.posx=in;
}
void SetPosY(double in)
{
  allPara.sDta.posy=in;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
