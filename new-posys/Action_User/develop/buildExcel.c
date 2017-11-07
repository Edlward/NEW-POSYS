/**
******************************************************************************
* @file     
* @author  lxy and Summer
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
#include "buildExcel.h"
#include "stdint.h"
#include "flash.h"
#include "config.h"
#include "timer.h"
#include "usart.h"
#include "temperature_control.h"
#include "icm_20608_g.h"
#include "arm_math.h"
#include "string.h"
#include "stm32f4xx_it.h"
#include "customer.h"
#include <stdlib.h>
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
extern double *chartWX;
extern double *chartWY;
extern double *chartWZ;
extern uint8_t 	*chartMode;
extern uint8_t 	*chartSelect;
extern uint8_t  *scaleMode;
extern float    *minValue;
extern float    *varXYZ;
void TempTablePrintf(void)
{
  Flash_Read(GetFlashArr(),TempTable_Num);
  
  USART_OUT(USART6,"chartWX:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*(chartWX+i));
  
  USART_OUT(USART6,"\r\nchartWY:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*(chartWY+i));
  
  USART_OUT(USART6,"\r\nchartWZ:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*(chartWZ+i));
  
  USART_OUT(USART6,"\r\nchartMode:\r\n%d",*chartMode);
  USART_OUT(USART6,"\r\nchartSelect:\r\n%d\t%d\t%d\t%d\t%d\r\n",*chartSelect,*(chartSelect+1),*(chartSelect+2),*(chartSelect+3),*(chartSelect+4));
  USART_OUT(USART6,"scaleMode: %d\r\n",*scaleMode);
  USART_OUT(USART6,"minValue:\r\n");
  USART_OUT_F(*(minValue));
	USART_OUT(USART6,"\r\nvarXYZ:  ");
  USART_OUT_F(*(varXYZ+0));
  USART_OUT_F(*(varXYZ+1));
  USART_OUT_F(*(varXYZ+2));
	USART_Enter();
  
}

extern gyro_t gyr_data;
extern float  temp_icm;
int CalculateCrAndMean(float stdCr[3],float mean[3]){
	static float IAE_st[3][200]={0.f};    //��¼����Ϣ
	static float data[3][200]={0.f};    	//������
	static int ignore=0;
	ignore++;
  /* �����б� */
	for(int i=0;i<3;i++){
		memcpy(data[i],data[i]+1,796);
	}
	data[0][199]=gyr_data.No1.x;
	data[1][199]=gyr_data.No1.y;
	data[2][199]=gyr_data.No1.z;
  /* ��Ϣ�ķ������ */
	for(int i=0;i<3;i++){
		memcpy(IAE_st[i],IAE_st[i]+1,796);
	}
	for(int i=0;i<3;i++)
	{
		mean[i]=mean[i]-data[i][0]/200+data[i][199]/200;
	}
	IAE_st[0][199]=gyr_data.No1.x-mean[0];
	IAE_st[1][199]=gyr_data.No1.y-mean[1];
	IAE_st[2][199]=gyr_data.No1.z-mean[2];
	
  if(ignore<400)
		return 0;
	else
		ignore=400;
	for(int j=0;j<3;j++){
		stdCr[j]=0;
		for(int i=0;i<200;i++)
		{
			stdCr[j]=stdCr[j]+IAE_st[j][i]*IAE_st[j][i];
		}
		stdCr[j]=__sqrtf(stdCr[j]/200.0f);
	}
	return 1;
}

int WaitForSlowDrift(void)
{
	static uint32_t time=0;
	time++;
	if(time<200*60*30)
		return 0;
	else
	{
		time=200*60*31;
		return 1;
	}
}

int UpdateVDoffTable(void)
{
	static uint32_t temp_count[(int)(TempTable_max-TempTable_min)*10]={0u};
	static long double temp_w[3][(int)(TempTable_max-TempTable_min)*10]={0.0};
  long double paraXY[3]={0.0};
  long double paraX    = 0.0 ;
  long double paraY[3] ={0.0};
  long double paraX2   = 0.0 ;
  long double temp_temp=temp_icm/100.f;
  int index=0;
  static float stdCr[3]={0.f};                //��Ϣ�ı�׼��
  static float mean[3]={0.f};
	
	if(!WaitForSlowDrift())
		return 0;
	
	/*���ҷ���*/
	if(!CalculateCrAndMean(stdCr,mean))
		return 0;
	
	if((fabs(gyr_data.No1.x-mean[0])>stdCr[0]*5)||(fabs(gyr_data.No1.y-mean[1])>stdCr[1]*5)||(fabs(gyr_data.No1.z-mean[2])>stdCr[2]*5))
	{
//		USART_OUT_F(temp_icm);
//		USART_OUT_F(gyr_data.No1.z);
		return 0;
	}
	else
	{	
//		USART_OUT_F(temp_icm);
//		USART_OUT_F(gyr_data.No1.z);
//		USART_Enter();	
	}		
	/*�����ص�0���������Ļ�����С��������ʵ����ֵ����*/
	if((double)temp_icm>=TempTable_min&&(double)temp_icm<TempTable_max-0.06){
	
		/*ȷ���¶�������,�������0.5���ܻ����index=200�����*/
		index=roundf(((double)temp_icm-TempTable_min)*10.0);
		if(temp_count[index]>0){
			/*����һ���¶��ϵĽ��ٶȺ�*/
			temp_w[0][index]+=gyr_data.No1.x;
			temp_w[1][index]+=gyr_data.No1.y;
			temp_w[2][index]+=gyr_data.No1.z;
		}
		else{
			/*��ʼֵ*/
			temp_w[0][index]=gyr_data.No1.x;
			temp_w[1][index]=gyr_data.No1.y;
			temp_w[2][index]=gyr_data.No1.z;
			temp_count[index]=0;
		}
			temp_count[index]++;
	}
	
	
  if(TempErgodic(0)==3){
		int32_t sum=0;
		USART_OUT(USART6,"\r\nfinish\r\n");
		for(int i=0;i<(TempTable_max-TempTable_min)*10;i++)
		{
			if(temp_count[i]>500){
				paraX=paraX+temp_count[i]*((long double)(TempTable_min+i/10.0)/100.0);
				paraX2  =paraX2 + temp_count[i]*(long double)((TempTable_min+i/10.0)/100.0)*(long double)((TempTable_min+i/10.0)/100.0);
				paraXY[0]=paraXY[0]+temp_w[0][i]*(long double)((TempTable_min+i/10.0)/100.0);
				paraXY[1]=paraXY[1]+temp_w[1][i]*(long double)((TempTable_min+i/10.0)/100.0);
				paraXY[2]=paraXY[2]+temp_w[2][i]*(long double)((TempTable_min+i/10.0)/100.0);
				paraY[0]=paraY[0]+temp_w[0][i];
				paraY[1]=paraY[1]+temp_w[1][i];
				paraY[2]=paraY[2]+temp_w[2][i];
				sum=sum+temp_count[i];
				
				temp_w[0][i]=temp_w[0][i]/temp_count[i];
				temp_w[1][i]=temp_w[1][i]/temp_count[i];
				temp_w[2][i]=temp_w[2][i]/temp_count[i];
				USART_OUT_F(TempTable_min+i/10.0);
				USART_OUT_F(temp_w[0][i]);
				USART_OUT_F(temp_w[1][i]);
				USART_OUT_F(temp_w[2][i]);
				USART_Enter();	
				delay_us(200);
			}else{
				temp_count[i]=0;
				temp_w[0][i]=0.f;
				temp_w[1][i]=0.f;
				temp_w[2][i]=0.f;
//				USART_OUT_F(TempTable_min+i/10.0);
//				USART_Enter();
			}
		}
    for(int i=4;i>0;i--){
      *(chartWX+i)=*(chartWX+i-1);
      *(chartWY+i)=*(chartWY+i-1);
      *(chartWZ+i)=*(chartWZ+i-1);
    }
		TempErgodic(1);
		//2*60/0.005=24000
    *chartWX=(sum*paraXY[0]-paraX*paraY[0])/(sum*paraX2-paraX*paraX);
    *chartWY=(sum*paraXY[1]-paraX*paraY[1])/(sum*paraX2-paraX*paraX);
    *chartWZ=(sum*paraXY[2]-paraX*paraY[2])/(sum*paraX2-paraX*paraX);
    Flash_Write(GetFlashArr(),TempTable_Num);
    USART_OUT(USART6,"Flash Update end\r\n");
    TempTablePrintf();
		{
			for(int i=0;i<3;i++){
				paraXY[i]=0.0;
				paraY[i]=0.0;
				paraX=0.0;
				paraX2=0.0;
			}
			for(int i=0;i<(int)(TempTable_max-TempTable_min)*10;i++)
			{
				temp_count[i]=0;
				temp_w[0][i]=0.f;
				temp_w[1][i]=0.f;
				temp_w[2][i]=0.f;
			}
		}
  }
	return 1;
}


void Hex_To_Str(uint8_t * pHex,char * s,float num)
{
	char        hex[] = "0123456789ABCDEF";
	char        *pStr = s;
	for (uint8_t i = 0; i < (int)(num/2.f+0.5f); i++)//(int)(x+0.5f)�ǰ�x�����������˼
	{
		
		/*
		1.*pStr++�ҽ��,����*��������û��++֮ǰ�ĵ�ַ
		2.f.��λ����ı�ָ��ָ����Ǹ��ռ��ֵ
		3.��ָ��ָ��ռ����λҲ����ı�ָ���ָ��
		*/
		if (((num<((int)(num / 2.f + 0.5f))*2.f)&&i>0)|| (num==((int)(num / 2.f + 0.5f)) * 2.f))
		*pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) >> 4];
		*pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) & 0x0F];
	}
}



/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
