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
extern float *chartWX;
extern float *chartWY;
extern float *chartWZ;
extern uint32_t *chartNum;
float KalmanT(float measureData);
float KalmanZ(float measureData);

void TempTablePrintf(void)
{
	float temp=TempTable_min;
	for(int i=0;i<TempTable_Num;i++)
	{
			USART_OUT_F(temp);
			USART_OUT_F(chartWX[i]);
			USART_OUT_F(chartWY[i]);
			USART_OUT_F(chartWZ[i]);
			USART_OUT(USART1,"\r\n");
			temp=temp+0.1f;
	}
}

void UpdateVDoffTable(void)
{
  gyro_t gyr_icm;
  float temp_icm;
	static double temp_w[3][TempTable_Num]={0.0};
	static uint32_t temp_count[TempTable_Num]={0u};
	static int index=0;
	static int time_count=0;
	
	static uint32_t coldCount=0;
	static const int time=200*60*3;
	coldCount++;
	if(coldCount<time){
		ICM_HeatingPower(0);
		USART_OUT(USART1,"waiting \r\n");
		return ;
	}
	else if(coldCount==time){
		Flash_Return();
	}else
		coldCount=time+1;
	
	icm_update_gyro_rate();
	icm_update_temp();
	
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	
	temp_icm=KalmanT(temp_icm);
	
	if(temp_icm>=TempTable_min&&temp_icm<TempTable_max){
		index=roundf((temp_icm-TempTable_min)*10);
		if(temp_count[index]>0&&temp_count[index]!=0xffffffff){
			temp_w[0][index]+=gyr_icm.No1.x;
			temp_w[1][index]+=gyr_icm.No1.y;
			temp_w[2][index]+=gyr_icm.No1.z;
		}
		else{
			temp_w[0][index]=gyr_icm.No1.x;
			temp_w[1][index]=gyr_icm.No1.y;
			temp_w[2][index]=gyr_icm.No1.z;
			temp_count[index]=0;
		}
			temp_count[index]++;
	}
	USART_OUT_F(gyr_icm.No1.x);
	USART_OUT_F(gyr_icm.No1.y);
	USART_OUT_F(gyr_icm.No1.z);
	USART_OUT_F(temp_icm);
	USART_Enter();
	time_count++;
	if(time_count>=2000)
	{
		time_count=0;
	}
	
	if(TempErgodic(TempTable_min,10,10)==3){
		for(int i=0;i<TempTable_Num;i++)
		{
				chartNum[i]=temp_count[i];
			if(chartNum[i]!=0xffffffff){
				chartWX[i]=(float)(temp_w[0][i]/temp_count[i]);
				chartWY[i]=(float)(temp_w[1][i]/temp_count[i]);
				chartWZ[i]=(float)(temp_w[2][i]/temp_count[i]);
			}
		}
		SetCommand(CORRECT);
		USART_OUT(USART1,"Flash Update begin\r\n");
		Flash_Write(GetFlashArr(),TempTable_Num*16);
		USART_OUT(USART1,"Flash Update end\r\n");
	  Flash_Read(GetFlashArr(),TempTable_Num*16);
		TempTablePrintf();
	}
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
