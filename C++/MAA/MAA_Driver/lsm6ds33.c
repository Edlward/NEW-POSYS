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
#include "lsm6ds33.h"
#include "spi.h"
#include "timer.h"
#include "usart.h"
#include "math.h"
#include "motion_attitude_algorithm.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float Temp;
static three_axis Gyr_w={0,0,0};
static three_axis ACC_g={0,0,0};
/* Extern   variables ---------------------------------------------------------*/
extern float K_acc;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief   
  * @none
  * @retval  
  */
void LSM6DS33_Init(void)
{
	#ifdef LSM6DS33_FIFO_ENABLE
		//软件重启
		LSM6D_Write(LSM6DS33_CTRL3_C,0x81);
	  LSM6D_Write(LSM6DS33_FIFO_CTRL5,0x40);
		TIM_Delayms(TIM1,50);
		LSM6D_Write(LSM6DS33_CTRL3_C,0x00);
		//重启完成
		
		LSM6D_Write(LSM6DS33_FUNC_CFG_ACCESS,0x00); 
		LSM6D_Write(LSM6DS33_FIFO_CTRL1,0xFF);
		LSM6D_Write(LSM6DS33_FIFO_CTRL2,0x0F);
		LSM6D_Write(LSM6DS33_FIFO_CTRL3,0x08);
		LSM6D_Write(LSM6DS33_FIFO_CTRL4,0x00);
		
		LSM6D_Write(LSM6DS33_FIFO_CTRL5,0x6E);
		
		LSM6D_Write(LSM6DS33_ORIENT_CFG_G,0x28);
		
		LSM6D_Write(LSM6DS33_CTRL1_XL,0x81);
		LSM6D_Write(LSM6DS33_CTRL2_G,0x82);
		LSM6D_Write(LSM6DS33_CTRL3_C,0x40);
		LSM6D_Write(LSM6DS33_CTRL4_C,0x85);
		LSM6D_Write(LSM6DS33_CTRL5_C,0x60);
		LSM6D_Write(LSM6DS33_CTRL6_C,0x00);
		LSM6D_Write(LSM6DS33_CTRL7_G,0x00);

		LSM6D_Write(LSM6DS33_CTRL8_XL,0x84);
		LSM6D_Write(LSM6DS33_CTRL9_XL,0x38);
		LSM6D_Write(LSM6DS33_CTRL10_C,0x38);

		LSM6D_Write(LSM6DS33_TAP_CFG,0X10);
	#else
	 	//软件重启
		LSM6D_Write(LSM6DS33_CTRL3_C,0xcd);
		TIM_Delayms(TIM1,50);
		LSM6D_Write(LSM6DS33_CTRL3_C,0x44);
		//重启完成
		
		LSM6D_Write(LSM6DS33_FUNC_CFG_ACCESS,0x00); 
		LSM6D_Write(LSM6DS33_FIFO_CTRL1,0xFF);
		LSM6D_Write(LSM6DS33_FIFO_CTRL2,0x0F);
		LSM6D_Write(LSM6DS33_FIFO_CTRL3,0x00);
		LSM6D_Write(LSM6DS33_FIFO_CTRL4,0x00);
		
		LSM6D_Write(LSM6DS33_FIFO_CTRL5,0x00);
		
		LSM6D_Write(LSM6DS33_ORIENT_CFG_G,0x28);
		
		LSM6D_Write(LSM6DS33_CTRL1_XL,0x81);
		LSM6D_Write(LSM6DS33_CTRL2_G,0x52);
		LSM6D_Write(LSM6DS33_CTRL3_C,0x04);
		LSM6D_Write(LSM6DS33_CTRL4_C,0x85);
		LSM6D_Write(LSM6DS33_CTRL5_C,0x60);
		LSM6D_Write(LSM6DS33_CTRL6_C,0x00);
		LSM6D_Write(LSM6DS33_CTRL7_G,0x00);

		LSM6D_Write(LSM6DS33_CTRL8_XL,0x60);
		LSM6D_Write(LSM6DS33_CTRL9_XL,0x38);
		LSM6D_Write(LSM6DS33_CTRL10_C,0x38);

	#endif
}
void LSM6DS33_UpdateTemp(void)
{
	uint8_t buf8[2];
	int16_t buf16;
	buf8[0]=LSM6D_Read(LSM6DS33_OUT_TEMP_L);
	buf8[1]=LSM6D_Read(LSM6DS33_OUT_TEMP_H);
  buf16=(short)((buf8[1]<<8)|buf8[0]);
	Temp=buf16/16.0+25;
}

void LSM6DS33_ReadTemp(float *temp)
{
	temp[0] =Temp;
}


void LSM6DS33_UpdateW(void)
{
	uint8_t buf8[2];
	int16_t buf16;
	
	three_axis temp;
	
	buf8[0]=LSM6D_Read(LSM6DS33_OUTX_L_G);
	buf8[1]=LSM6D_Read(LSM6DS33_OUTX_H_G);
	buf16=(short)((buf8[1]<<8)|buf8[0]);
	temp.x=buf16*0.004375;
	
	buf8[0]=LSM6D_Read(LSM6DS33_OUTY_L_G);
	buf8[1]=LSM6D_Read(LSM6DS33_OUTY_H_G);
	buf16=(short)((buf8[1]<<8)|buf8[0]);
	temp.y=buf16*0.004375;
	
	
	buf8[0]=LSM6D_Read(LSM6DS33_OUTZ_L_G);
	buf8[1]=LSM6D_Read(LSM6DS33_OUTZ_H_G);
	buf16=(short)((buf8[1]<<8)|buf8[0]);
	temp.z=buf16*0.004375;
	
	Gyr_w.x=-temp.y;
	Gyr_w.y= temp.x;
	Gyr_w.z=-temp.z;
}
void LSM6DS33_UpdateACC(void)
{
	uint8_t buf8[2];
	int16_t buf16;
	buf8[0]=LSM6D_Read(LSM6DS33_OUTX_L_XL);
	buf8[1]=LSM6D_Read(LSM6DS33_OUTX_H_XL);
	buf16=(short)((buf8[1]<<8)|buf8[0]);
	ACC_g.x=buf16*0.061;
	
	buf8[0]=LSM6D_Read(LSM6DS33_OUTY_L_XL);
	buf8[1]=LSM6D_Read(LSM6DS33_OUTY_H_XL);
	buf16=(short)((buf8[1]<<8)|buf8[0]);
	ACC_g.y=buf16*0.061;
	
	buf8[0]=LSM6D_Read(LSM6DS33_OUTZ_L_XL);
	buf8[1]=LSM6D_Read(LSM6DS33_OUTZ_H_XL);
	buf16=(short)((buf8[1]<<8)|buf8[0]);
	ACC_g.z=buf16*0.061;
	
}
void LSM6D_ReadGyr(three_axis *gyr)
{
	(*gyr).x= Gyr_w.y;
	(*gyr).y= Gyr_w.x;
	(*gyr).z= Gyr_w.z;
}	
void LSM6D_ReadAcc(three_axis *acc)
{
	(*acc).x=ACC_g.x;
	(*acc).y=ACC_g.y;  
	(*acc).z=ACC_g.z;
}
void LSM6D_ReadAccRad(three_axis *rad)
{
	float sum=sqrt(ACC_g.x*ACC_g.x+ACC_g.y*ACC_g.y+ACC_g.z*ACC_g.z);
	float X_G,Y_G,Z_G;
	static uint8_t fix_flag=0;
	static float fix_sum;
	
	
	if(fix_flag<10)
	{
		fix_sum=fix_sum+sum/10;
		fix_flag++;
	}
	else if(fabs(sum-fix_sum)<1)
   K_acc =  0.98;
	else if(fabs(sum-fix_sum)<2)
   K_acc = 0.985;
	else if(fabs(sum-fix_sum)<3)
   K_acc = 0.987;
	else if(fabs(sum-fix_sum)<4)
   K_acc = 0.988;
	else if(fabs(sum-fix_sum)<10)
	 K_acc = 0.989;
	else 
	 K_acc=1;	
	
	X_G=(ACC_g).x/sum;
	Y_G=(ACC_g).y/sum;
	Z_G=(ACC_g).z/sum;
	
	
	(*rad).x=safe_atan2( X_G , -Z_G);
  (*rad).y=safe_atan2( Y_G , X_G/sin((*rad).x));
}
BOOL LSM6D_FIFO_Read(LSM6D_FIFO *data)
{
	uint8_t FIFO_Status1;
	uint8_t FIFO_Status2;
  uint8_t FIFO_Status3;
	uint8_t FIFO_Status4;
	uint8_t bufL;
	uint8_t bufH;
	uint16_t buf16;
  uint16_t i;
  uint16_t NextRead;
	
	uint8_t count[3]={0,0,0};
	
	FIFO_Status1=LSM6D_Read(LSM6DS33_FIFO_STATUS1);
	FIFO_Status2=LSM6D_Read(LSM6DS33_FIFO_STATUS2);

	
	(*data).Len=FIFO_Status1|((FIFO_Status2&0x0f)<<8);
	(*data).FTH=(FIFO_Status2&0X80)>>7;
	(*data).OR=(FIFO_Status2&0X40)>>6;
	(*data).Full=(FIFO_Status2&0X20)>>5;
	(*data).Empty=(FIFO_Status2&0X10)>>4;
	for(i=0;i<16;i++)
	{
		(*data).wx[i]=0;
		(*data).wy[i]=0;
		(*data).wz[i]=0;
	}
	for(i=0;(i<((*data).Len))&&(i<48);i++)
	{
	  FIFO_Status3=LSM6D_Read(LSM6DS33_FIFO_STATUS3);
	  FIFO_Status4=LSM6D_Read(LSM6DS33_FIFO_STATUS4);
		
		NextRead=((FIFO_Status4&0X03)<<8)|FIFO_Status3;
		
		bufL=LSM6D_Read(LSM6DS33_FIFO_DATA_OUT_L);
		bufH=LSM6D_Read(LSM6DS33_FIFO_DATA_OUT_H);
		buf16=(bufH<<8)|bufL;
		switch(NextRead)
		{
			case 0:
		   (*data).wx[count[0]]=((short)buf16)*0.004375;
			 count[0]++;
			 break;
			case 1:
		   (*data).wy[count[1]]=((short)buf16)*0.004375;
			 count[1]++;
			 break;
			case 2:
			 (*data).wz[count[2]]=((short)buf16)*0.004375;
			 count[2]++;
			 break;
      default:
				return FALSE;
		}
		if((count[0]==16)||(count[1]==16)||(count[2]==16))
		{
			return TRUE;
		}
	}
	return TRUE;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/





