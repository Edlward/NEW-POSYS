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
25.18 25.19�øĽ��㷨����ϵó���
�ֶ���	��x����Ϊ��׼  
��һ25.1606371025683  �ֶ�25.2220631351932
	//1/4096*wheelR*2*pi
	allPara.sDta.posx=convert_X;
	allPara.sDta.posy=convert_Y;
	a=-0.1304/180*pi;��õ�������
  real=[1 0;-tan(a) 1/cos(a)]*[allPara.sDta.vell1';allPara.sDta.vell2'];
	real[0]=allPara.vell[0]*0.0385959339263024;
  real[1]=0.00227591327416601*allPara.vell[0]*0.0385959339263024+1.00000258988726*allPara.vell[1]*0.038690160280225;
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
	/*-180��180*/
	if(zangle-last_ang>180.0)
		zangle=(zangle-360.0+last_ang)/2.0;
	if(zangle-last_ang<-180.0)
		zangle=(zangle+360.0+last_ang)/2.0;
	
	if(zangle>180.0)
		zangle-=360.0;
	else if(zangle<-180.0)
		zangle+=360.0;
	
	last_ang=allPara.sDta.Result_Angle[2];
	
	//ֱ������ϵ�ͷ�ֱ������ϵ��ת��  һ��Ҫע������ϵ��������ͽǶ�������һ����

	#ifdef AUTOCAR	//��yΪ��׼
		real[0]=allPara.sDta.vellF[0];
		// һ�Ż��Ӽ� 1.00005377860061 -0.0103711182308368
		real[1]=1.00005377860061*(double)allPara.sDta.vellF[1]-0.0103711182308368*(double)allPara.sDta.vellF[0];
	#else			//��xΪ��׼
		real[0]=allPara.sDta.vellF[0];
		real[1]=allPara.sDta.vellF[1];
	#endif

	delPos[0]=(sin(zangle*0.017453292519943)*real[1]+cos(zangle*0.017453292519943)*real[0]);
	delPos[1]=(cos(zangle*0.017453292519943)*real[1]-sin(zangle*0.017453292519943)*real[0]);

	allPara.sDta.posx+=delPos[0];
	allPara.sDta.posy+=delPos[1];
	
	/*��ö�λϵͳx��y�����ϵ��ٶ�*/
	allPara.sDta.vellx=delPos[0]*200.f;
	allPara.sDta.velly=delPos[1]*200.f;
	
}

void figureVell(void)
{
	static uint8_t flag=0;
	
	#ifdef TLE5012_USED
		//��һ���жϾ�ֹʱ���Բ����жϽ��ٶ�
		allPara.sDta.codeData[0]=TLE5012ReadAbsPos_A();
		allPara.sDta.codeData[1]=TLE5012ReadAbsPos_B();
	#else
		//��һ���жϾ�ֹʱ���Բ����жϽ��ٶ�
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
		
		// һ�Ż��Ӽ� 0.0387225283845694 0.0387374461979914
		// ���Ż��Ӽ� 0.0387225283845694 0.0387374461979914
		allPara.sDta.vellF[0]=allPara.vell[0]*0.0387225283845694;
		allPara.sDta.vellF[1]=allPara.vell[1]*0.0387374461979914;
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
