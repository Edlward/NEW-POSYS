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
#include "vdoff.h"
#include "stdint.h"
#include "flash.h"
#include "maa_config.h"
#include "timer.h"
#include "usart.h"
#include "temperature_control.h"
#include "icm_20608_g.h"
#include "lsm6ds33.h"
#include "math.h"
#include "string.h"
#include "action_math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float *VDoff;
static uint32_t *countnum_vdoff;
static float **a_BasicFitting;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/**
  * @brief       �¶�Ư�Ʊ���
  * @param[in]   
  * @retval None
  */
static void Table_VDoff_Temp(three_axis ICM_Gyr,  float ICM_Temp,
											       float *result,       uint32_t *count_num)
{
  	int16_t convert;
	  float data;
	  uint32_t count;
				
		convert=(int16_t)((ICM_Temp-TempTable_min)*10)/1;
	  if(convert>=0&&convert<10*(TempTable_max-TempTable_min))
		{
			/*
			�Դ��ڴ��¶��µ�������ٶ�ȡһ��ƽ����
			*/
			data=*(result+convert);
			count=*(count_num+convert);
			*(result+convert)=data*((float)count/(count+1))+ICM_Gyr.x/(count+1);
			data=*(result+convert+10*(TempTable_max-TempTable_min));
			*(result+10*(TempTable_max-TempTable_min)+convert)=data*((float)count/(count+1))+ICM_Gyr.y/(count+1);
			data=*(result+convert+20*(TempTable_max-TempTable_min));
			*(result+20*(TempTable_max-TempTable_min)+convert)=data*((float)count/(count+1))+ICM_Gyr.z/(count+1);
			/*
			���¶ȼ������ټ�һ
			*/
			*(count_num+convert)=(*(count_num+convert))+1;
	   }
}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void WaitForUpdataVDoff(void)
{
  uint32_t count;
	three_axis gyr_icm_read;
	float temp_icm;
	/*
	static float *VDoff;
	return result;
  static float    *Result;
	Result=(float *)flashdata;
	static uint8_t flashdata[160*(TempTable_max-TempTable_min)];
	Flash_Read(flashdata,16*10*(TempTable_max-TempTable_min));
	*/
	VDoff=GetResultArr();
	/*
	static uint32_t *countnum_vdoff;
	return countnum;
	countnum=(uint32_t *)(flashdata)+30*(TempTable_max-TempTable_min);
	*/
	countnum_vdoff=GetCountArr();
	/*
	#define BF_TH          2               //���¶��˷���ϵĽ���
	��������������,2*2��A��2*3��Y�����
	2*2Ϊ�����,2*3Ϊ�Ҿ���(��Ϊ������,����������)
	*/
	BasicFitting_th(BF_TH);
	
	float temp=TempTable_min;//10
	/*
	�ܹ���40*(TempTable_max-TempTable_min)��32λ
	�˴�ȡ����10*(TempTable_max-TempTable_min)��32λ
	*/
	for(uint32_t i=0;i<10*(TempTable_max-TempTable_min);i++)
	{
		if(temp>(TempTable_min-1)&&temp<(TempTable_max+1))
		{
			//����ֵҪ����10��������,����ʱ��0xffffffff,flash������
			if(countnum_vdoff[i]>10 &&
				 countnum_vdoff[i]!=0xffffffff)
			{
				/*
				��С���˷�������������ֹ���,
				һ��flash������Ʈ����,
				���ǳ�ʼ���ɼ�������Ʈ����,
				����flash������ռ�������,
				�Լ�Ȩ�ķ�ʽ�Զ��ߵ���С���˷����н��
				*/
				for(uint32_t j=0;j<TIME_HEAT*200;j++)
				{
					//�����������ֱ���y,x,z����Ʈ���ٶ�
					ordata_input(temp,(*(VDoff+i)),
					                  (*(VDoff+i+10*(TempTable_max-TempTable_min))),
					                  (*(VDoff+i+20*(TempTable_max-TempTable_min))));
				}
			}
		}
		temp=temp+0.1f;
	}
	
	for(count=0;count<TIME_HEAT*200;count++)
	{
		/* 
		����ʱ������ 
		����һ������ִֻ��һ��
		*/
		while(!get_time_sig());
		temperature_control(Temp_ctr);
		icm_update_temp();
		icm_update_gyro_rate();
		icm_read_gyro_rate(&gyr_icm_read);
		icm_read_temp(&temp_icm);
		ordata_input(temp_icm,gyr_icm_read.x,gyr_icm_read.y,gyr_icm_read.z);
	}
  //static float **a_BasicFitting;
  a_BasicFitting=BasicFittingCal();
}

void TempTablePrintf(float *result)
{
	float temp=TempTable_min;
	uint32_t i;
	
	for(i=0;i<10*(TempTable_max-TempTable_min);i++)
	{
		if(temp>(TempTable_min-1)&&temp<(TempTable_max+1))
		{
			USART_OUT(USART1,"�¶� %d\r\n",(int)(temp*100));
			USART_OUT(USART1,"X    %d\r\n",(int)(*(result+i+0)*10000));
			USART_OUT(USART1,"Y    %d\r\n",(int)(*(result+i+10*(TempTable_max-TempTable_min))*10000));
			USART_OUT(USART1,"Z    %d\r\n",(int)(*(result+i+20*(TempTable_max-TempTable_min))*10000));
		}
		temp=temp+0.1f;
	}
}
float *GetVdoff_icmArr()
{
	return VDoff;
}
float **GetVdoff_icmErrArr()
{
	return a_BasicFitting;
}
uint32_t *GetCountnum_icmArr()
{
	return countnum_vdoff;
}
void UpdateVDoffTable(void)
{
	static three_axis gyr_icm;
	static float temp_icm;
	
	static float    *Result;
	static uint32_t *countnum;
	
	static int circle_count;
	static int time_count=0;
	
	static uint32_t flag_wait_SetMin=0;
	static uint8_t Min_TempTable=0;
	/*flash.c
	static float    *Result;
	static uint32_t *countnum;
	*/
	Result=GetResultArr();
	countnum=GetCountArr();
	
	icm_update_gyro_rate();
	icm_update_temp();
	
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);

  if(flag_wait_SetMin>200*60*7)
	{
		Table_VDoff_Temp(gyr_icm,temp_icm,
										 Result, countnum);
		circle_count++;								 
		if(circle_count/12000+Min_TempTable<=TempTable_max)
			temperature_control(circle_count/12000+Min_TempTable);
		/*��������¶Ⱥ��ٷ�����*/
		else if(TempTable_max+TempTable_max-circle_count/12000-Min_TempTable>=Min_TempTable)
			temperature_control(TempTable_max+TempTable_max-circle_count/12000-Min_TempTable);
		else
			circle_count=0;
		
		time_count++;
		if(time_count>2000)
		{
			TempTablePrintf(GetResultArr());
			Flash_Write(GetFlashArr(),160*(TempTable_max-TempTable_min));
			USART_OUT(USART1,"ACT TEMP: %d\r\n",(int)(temp_icm*100));
			USART_OUT(USART1,"TempMin: %d\r\n",(int)Min_TempTable);
			USART_OUT(USART1,"Flash Update\r\n");
			time_count=0;
		}
	}
	else if(flag_wait_SetMin==200*60*7)
	{
		flag_wait_SetMin++;
		Min_TempTable=(uint8_t)temp_icm;
		USART_OUT(USART1,"wait Ok\r\n");
		USART_OUT(USART1,"TempMin: \r\n",(int)Min_TempTable);
	}
	else
	{
		ICM_HeatingPower(0);
		USART_OUT(USART1,"wait for initialization\r\n");
		flag_wait_SetMin++;
	}
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
