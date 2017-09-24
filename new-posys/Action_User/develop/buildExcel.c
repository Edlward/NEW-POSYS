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
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
extern float *chartW;
extern uint32_t *chartNum;
float KalmanT(float measureData);
float KalmanZ(float measureData);

/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void UpdataExcel(void)
{
  while()
}

void TempTablePrintf(float *result)
{
	float temp=30.f;
	uint32_t i;
	Flash_Read(GetFlashArr(),(4+4)*(int)(1.f/TempStep)*(TempTable_max-TempTable_min));
	for(i=0;i<(int)((TempTable_max-TempTable_min)/TempStep);i++)
	{
			USART_OUT_F(temp);
			USART_OUT_F(chartW[i]);
			USART_OUT_F(chartNum[i]);
			temp=temp+TempStep;
			USART_OUT(USART1,"\r\n");
	}
}

void UpdateVDoffTable(void)
{
	static gyro_t gyr_icm;
	static float temp_icm;
	static uint32_t index=0;
	static int time_count=0;
	
	static uint32_t coldCount=0;
	static const int time=200*60*5;
	if(coldCount++<time)
		return ;
	else
		coldCount=time;
	
	icm_update_gyro_rate();
	icm_update_temp();
	
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	
	if(temp_icm>=TempTable_min){
		index=(uint32_t)((temp_icm-TempTable_min)/TempStep);
		if(chartNum[index]>0){
			chartW[index]=( chartW[index] * chartNum[index] + gyr_icm.No1.z ) / ( chartNum[index] + 1 );
		}
		else{
			chartW[index]=gyr_icm.No1.z;
		}
			chartNum[index]++;
	}
	
	time_count++;
	if(time_count>=2000)
	{
		USART_OUT(USART1,"ACT TEMP: %d\r\n",(int)(temp_icm*100));
		time_count=0;
	}
	
	if(TempErgodic(30,20,20)==3){
		SetFlashUpdateFlag(0);
		USART_OUT(USART1,"Flash Update begin\r\n");
		Flash_Write(GetFlashArr(),160*(TempTable_max-TempTable_min));
		USART_OUT(USART1,"Flash Update end\r\n");
		TempTablePrintf(chartW);
	}
}

/*�㷨�� H,��,����Ϊһ*/
float KalmanZ(float measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.007;       //ϵͳ����         
	static double R=0.007f;      //�������� 
	static float IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
	predict=act_value;
	
	/* ��Ϣ�ķ������ */
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;
	
	static uint8_t ignore=0;

	if(ignore++<50)
		return measureData;
	else
		ignore=50;
	
	Cr=0;
	for(int i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/50.0f;
	
	
	/* Ԥ�Ȿ�ε�Ԥ����� */
	P_mid=P_last+Q;
	
	/* ����ϵ����������ֵ */
	Kk=P_mid/(P_mid+R);
	
	act_value=predict+Kk*(measureData-predict);
	
	/* ����Ԥ����� */
	P_last=(1-Kk)*P_mid;
	
	/* ���㲢����ϵͳ���� */
	Q=Kk*Kk*Cr;

	/* Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ */
	if(Kk>0.5)
		act_value=measureData;
	
	return act_value;
}

float KalmanT(float measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.025;       //ϵͳ����
	static double R=0.025;      //��������         
	
	static float IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
	/* ��������û���У��ֵ������û�����������ʹ��0.012 */
	//R=Get_R_Zaxis();
   
	predict=act_value;
	
	//�൱�����������
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;
	
	static uint8_t ignore=0;

	if(ignore++<50)
		return measureData;
	else
		ignore=50;
	
	/* ��Ϣ�ķ������ */
	Cr=0;
	for(int i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/50.0f;		//���������֪���Բ��ԣ���Ҫ��ԭ��ʽ��
	
	/* Ԥ�Ȿ�ε�Ԥ����� */
	P_mid=P_last+Q;
	
	/* ����ϵ����������ֵ */
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(measureData-predict);
	
	/* ����Ԥ����� */
	P_last=(1-Kk)*P_mid;
	
	/* ���㲢����ϵͳ���� */
	Q=Kk*Kk*Cr;

	/* Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ */
	if(Kk>0.5)
		act_value=measureData;
	
	return act_value;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
