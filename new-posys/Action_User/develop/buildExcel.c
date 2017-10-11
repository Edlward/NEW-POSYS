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
extern gyro_t gyr_icm;
extern float  temp_icm;
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
  
  USART_OUT(USART1,"chartMode:\r\n%d",*chartMode);
  USART_OUT(USART1,"chartSelect:\r\n%d\t%d\t%d\t%d\t%d\r\n",*chartSelect,*(chartSelect+1),*(chartSelect+2),*(chartSelect+3),*(chartSelect+4));
  USART_OUT(USART1,"scaleMode: %d\r\n",*scaleMode);
  USART_OUT(USART1,"minValue:\r\n");
  USART_OUT_F(*(minValue));
	USART_OUT(USART1,"\r\nvarXYZ:  ");
  USART_OUT_F(*(varXYZ+0));
  USART_OUT_F(*(varXYZ+1));
  USART_OUT_F(*(varXYZ+2));
	USART_Enter();
  
}

void UpdateVDoffTable(void)
{
  static long double paraXY[3]={0.0};
  static long double paraX    = 0.0 ;
  static long double paraY[3] ={0.0};
  static long double paraX2   = 0.0 ;
  static uint32_t time_count=0;
  float temp_temp=temp_icm/100.f;
  time_count++;
  if(time_count<=3.5*60*200&&time_count>=0.25*60*200){
    
		ICM_HeatingPower(getHeatPower());
    paraX   =paraX + (double)temp_temp;
    
    paraX2  =paraX2 + (double)temp_temp*(double)temp_temp;
    
    paraY[0]=paraY[0]+(double)gyr_icm.No1.x;
    paraY[1]=paraY[1]+(double)gyr_icm.No1.y;
    paraY[2]=paraY[2]+(double)gyr_icm.No1.z;
    
    paraXY[0]=paraXY[0]+(double)gyr_icm.No1.x*(double)temp_temp;
    paraXY[1]=paraXY[1]+(double)gyr_icm.No1.y*(double)temp_temp;
    paraXY[2]=paraXY[2]+(double)gyr_icm.No1.z*(double)temp_temp;
    USART_OUT_F(temp_temp);
    USART_OUT_F(gyr_icm.No1.x);
    USART_OUT_F(gyr_icm.No1.y);
    USART_OUT_F(gyr_icm.No1.z);
    USART_Enter();
  }
  else if(time_count==(3.5*60*200+1)){
    for(int i=4;i>0;i--){
      *(chartWX+i)=*(chartWX+i-1);
      *(chartWY+i)=*(chartWY+i-1);
      *(chartWZ+i)=*(chartWZ+i-1);
    }
    *chartWX=(39000.0*paraXY[0]-paraX*paraY[0])/(39000.0*paraX2-paraX*paraX);
    *chartWY=(39000.0*paraXY[1]-paraX*paraY[1])/(39000.0*paraX2-paraX*paraX);
    *chartWZ=(39000.0*paraXY[2]-paraX*paraY[2])/(39000.0*paraX2-paraX*paraX);
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
			time_count=0;
		}
  }else if(time_count>(3.5*60*200+1))
  {
    time_count=666666;
  }
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
