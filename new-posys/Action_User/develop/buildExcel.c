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
  
  USART_OUT(USART1,"chartWX:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*(chartWX+i));
  
  USART_OUT(USART1,"\r\nchartWY:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*(chartWY+i));
  
  USART_OUT(USART1,"\r\nchartWZ:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*(chartWZ+i));
  
  USART_OUT(USART1,"\r\nchartMode:\r\n%d",*chartMode);
  USART_OUT(USART1,"\r\nchartSelect:\r\n%d\t%d\t%d\t%d\t%d\r\n",*chartSelect,*(chartSelect+1),*(chartSelect+2),*(chartSelect+3),*(chartSelect+4));
  USART_OUT(USART1,"scaleMode: %d\r\n",*scaleMode);
  USART_OUT(USART1,"minValue:\r\n");
  USART_OUT_F(*(minValue));
	USART_OUT(USART1,"\r\nvarXYZ:  ");
  USART_OUT_F(*(varXYZ+0));
  USART_OUT_F(*(varXYZ+1));
  USART_OUT_F(*(varXYZ+2));
	USART_Enter();
  
}

extern gyro_t gyr_icm;
extern float  temp_icm;
void UpdateVDoffTable(void)
{
	static long double temp_w[3][(TempTable_max-TempTable_min)*10]={0.0};
	static uint32_t temp_count[(TempTable_max-TempTable_min)*10]={0u};
  long double paraXY[3]={0.0};
  long double paraX    = 0.0 ;
  long double paraY[3] ={0.0};
  long double paraX2   = 0.0 ;
  int index=0;
  long double temp_temp=temp_icm/100.f;
	
	if(temp_icm>=TempTable_min&&temp_icm<TempTable_max){
		/*确定温度索引号*/
		index=roundf((temp_icm-TempTable_min)*10);
		if(temp_count[index]>0){
			/*求这一个温度上的角速度和*/
			temp_w[0][index]+=gyr_icm.No1.x;
			temp_w[1][index]+=gyr_icm.No1.y;
			temp_w[2][index]+=gyr_icm.No1.z;
		}
		else{
			/*初始值*/
			temp_w[0][index]=gyr_icm.No1.x;
			temp_w[1][index]=gyr_icm.No1.y;
			temp_w[2][index]=gyr_icm.No1.z;
			temp_count[index]=0;
		}
			temp_count[index]++;
	}
	
    USART_OUT_F(temp_temp);
    USART_OUT_F(gyr_icm.No1.x);
    USART_OUT_F(gyr_icm.No1.y);
    USART_OUT_F(gyr_icm.No1.z);
    USART_Enter();
  if(TempErgodic()==3){
		USART_OUT(USART1,"\r\nchart\r\n\r\n");
		for(int i=0;i<(TempTable_max-TempTable_min)*10;i++)
		{
			paraX=paraX+temp_count[i]*((TempTable_min+i/10.f)/100.f);
			paraX2  =paraX2 + temp_count[i]*(double)((TempTable_min+i/10.f)/100.f)*(double)((TempTable_min+i/10.f)/100.f);
			paraXY[0]=paraXY[0]+temp_w[0][i]*(double)((TempTable_min+i/10.f)/100.f);
			paraXY[1]=paraXY[1]+temp_w[1][i]*(double)((TempTable_min+i/10.f)/100.f);
			paraXY[2]=paraXY[2]+temp_w[2][i]*(double)((TempTable_min+i/10.f)/100.f);
		  paraY[0]=paraY[0]+temp_w[0][i];
      paraY[1]=paraY[1]+temp_w[1][i];
      paraY[2]=paraY[2]+temp_w[2][i];
			if(temp_count[i]!=0x00){
					temp_w[0][i]=(float)(temp_w[0][i]/temp_count[i]);
					temp_w[1][i]=(float)(temp_w[1][i]/temp_count[i]);
					temp_w[2][i]=(float)(temp_w[2][i]/temp_count[i]);
			}
			USART_OUT_F(temp_temp);
			USART_OUT_F(temp_w[0][i]);
			USART_OUT_F(temp_w[1][i]);
			USART_OUT_F(temp_w[2][i]);
			USART_Enter();
		}
    for(int i=4;i>0;i--){
      *(chartWX+i)=*(chartWX+i-1);
      *(chartWY+i)=*(chartWY+i-1);
      *(chartWZ+i)=*(chartWZ+i-1);
    }
		//2*60/0.005=24000
    *chartWX=(HEATTIME*24000.0*paraXY[0]-paraX*paraY[0])/(HEATTIME*24000.0*paraX2-paraX*paraX);
    *chartWY=(HEATTIME*24000.0*paraXY[1]-paraX*paraY[1])/(HEATTIME*24000.0*paraX2-paraX*paraX);
    *chartWZ=(HEATTIME*24000.0*paraXY[2]-paraX*paraY[2])/(HEATTIME*24000.0*paraX2-paraX*paraX);
    Flash_Write(GetFlashArr(),TempTable_Num);
    USART_OUT(USART1,"Flash Update end\r\n");
    TempTablePrintf();
		SetCommand(~CORRECT);
		ICM_HeatingPower(0);
		{
			for(int i=0;i<3;i++){
				paraXY[i]=0.0;
				paraY[i]=0.0;
				paraX=0.0;
				paraX2=0.0;
			}
		}
  }
	
}


void Hex_To_Str(uint8_t * pHex,char * s,float num)
{
	char        hex[] = "0123456789ABCDEF";
	char        *pStr = s;
	for (uint8_t i = 0; i < (int)(num/2.f+0.5f); i++)//(int)(x+0.5f)是把x四舍五入的意思
	{
		
		/*
		1.*pStr++右结合,并且*索引的是没有++之前的地址
		2.f.移位不会改变指针指向的那个空间的值
		3.对指针指向空间的移位也不会改变指针的指向
		*/
		if (((num<((int)(num / 2.f + 0.5f))*2.f)&&i>0)|| (num==((int)(num / 2.f + 0.5f)) * 2.f))
		*pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) >> 4];
		*pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) & 0x0F];
	}
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
