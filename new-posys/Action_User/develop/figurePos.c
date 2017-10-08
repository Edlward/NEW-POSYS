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
#include "figurePos.h"
#include "stdint.h"
#include "figureAngle.h"
#include "config.h"
#include "math.h"
#include "usart.h"
#include "spi.h"
#include "arm_math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float posx=0,posy=0;
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/


void calculatePos(void)
{
  static uint32_t data_last[2]={0,0};
  static uint32_t flag=0;
  
  double  pos_temp[2]={0,0};
  
  uint32_t data[2];
  int32_t vell[2];
  
  float zangle;
  static float last_ang=0.0f;
  
  data[0]=SPI_ReadAS5045(0);
  data[1]=SPI_ReadAS5045(1);
  
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
  
  zangle=getAngle().z;
  zangle=(zangle+last_ang)/2.0f;
  last_ang=getAngle().z;
	
	pos_temp[0]=vell[0]/4096.0*R_wheel1*2.0*PI;
	pos_temp[1]=vell[1]/4096.0*R_wheel2*2.0*PI;
	
	//-0.6677
	pos_temp[1]=(pos_temp[1]-pos_temp[0]*sin(-0.011653563415566))/cos(-0.011653563415566);
 
  posx+=(-sin(((double)zangle+45.0)/180.0*PI)*pos_temp[1]+cos(((double)zangle+45.0)/180.0*PI)*pos_temp[0]);
  posy+=(+cos(((double)zangle+45.0)/180.0*PI)*pos_temp[1]+sin(((double)zangle+45.0)/180.0*PI)*pos_temp[0]);
  
//	USART_OUT_F(posx);
//	USART_OUT_F(posy);
//	USART_Enter();
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
