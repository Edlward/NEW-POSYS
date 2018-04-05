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
#include "customer.h"
#include "motion_attitude_algorithm.h"
#include "string.h"
#include "stm32f4xx_usart.h"
#include "pos.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float R=0.012;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void Set_R_Zaxis(float val)
{
	R=val;
}
float Get_R_Zaxis(void)
{
	return R;
}
void DataSend(void)
{
#ifdef BIGCAR	
	int i;
	uint8_t 
	tdata[28];
	three_axis angle;
	three_axis w_icm;
  union{
		float   val;
		uint8_t data[4];
	}valSend;
	angle=getAngle();
	
  tdata[0]=0x0d;
  tdata[1]=0x0a;
  tdata[26]=0x0a;
  tdata[27]=0x0d;
	
	valSend.val=angle.z;
  memcpy(tdata+2,valSend.data,4);
	
	valSend.val=angle.x;
  memcpy(tdata+6,valSend.data,4);
	
	valSend.val=angle.y;
  memcpy(tdata+10,valSend.data,4);
	
	valSend.val=getPosX();
  memcpy(tdata+14,valSend.data,4);
	 
	valSend.val=getPosY();
  memcpy(tdata+18,valSend.data,4);
	 
	icm_read_gyro_rate(&w_icm);
	valSend.val=w_icm.z;
  memcpy(tdata+22,valSend.data,4);
	
	for(i=0;i<28;i++)
   USART_SendData(USART3,tdata[i]);	
#endif
#ifdef SMALLCAR
	union
	{
		uint8_t data[4];
		float actVal;
	}Trans;
	uint8_t tdata[8]={0x0a,0x0b,0x0c,0x00,
										0x00,0x00,0x00,0x00};
  static three_axis euler_angle={0,0,0};
           
  euler_angle=getAngle();    
	
  Trans.actVal=euler_angle.z;
  memcpy(tdata,Trans.data,4);
  tdata[4]=0x11;
  CAN_TxMsg(CAN1,0x12,tdata,5);
	   
  Trans.actVal=euler_angle.x;
  memcpy(tdata,Trans.data,4);
  tdata[4]=0x12;
  CAN_TxMsg(CAN1,0x12,tdata,5);
	  
  Trans.actVal=euler_angle.y;
  memcpy(tdata,Trans.data,4);
  tdata[4]=0x13;
  CAN_TxMsg(CAN1,0x12,tdata,5);

#endif
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
