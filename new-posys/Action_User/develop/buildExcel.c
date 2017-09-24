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

/*算法中 H,φ,Γ均为一*/
float KalmanZ(float measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.007;       //系统噪声         
	static double R=0.007f;      //测量噪声 
	static float IAE_st[50];    //记录的新息
	double Cr=0;                //新息的方差
	
  //令预测值为上一次的真实值
	predict=act_value;
	
	/* 新息的方差计算 */
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
	
	
	/* 预测本次的预测误差 */
	P_mid=P_last+Q;
	
	/* 计算系数，求得输出值 */
	Kk=P_mid/(P_mid+R);
	
	act_value=predict+Kk*(measureData-predict);
	
	/* 更新预测误差 */
	P_last=(1-Kk)*P_mid;
	
	/* 计算并调整系统噪声 */
	Q=Kk*Kk*Cr;

	/* 为提高滤波器的响应速度，减小滞后而设下的阈值 */
	if(Kk>0.5)
		act_value=measureData;
	
	return act_value;
}

float KalmanT(float measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.025;       //系统噪声
	static double R=0.025;      //测量噪声         
	
	static float IAE_st[50];    //记录的新息
	double Cr=0;                //新息的方差
	
	/* 获得来自用户的校正值，如果用户不矫正，则使用0.012 */
	//R=Get_R_Zaxis();
   
	predict=act_value;
	
	//相当于数组的左移
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;
	
	static uint8_t ignore=0;

	if(ignore++<50)
		return measureData;
	else
		ignore=50;
	
	/* 新息的方差计算 */
	Cr=0;
	for(int i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/50.0f;		//样本方差（不知道对不对，还要看原公式）
	
	/* 预测本次的预测误差 */
	P_mid=P_last+Q;
	
	/* 计算系数，求得输出值 */
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(measureData-predict);
	
	/* 更新预测误差 */
	P_last=(1-Kk)*P_mid;
	
	/* 计算并调整系统噪声 */
	Q=Kk*Kk*Cr;

	/* 为提高滤波器的响应速度，减小滞后而设下的阈值 */
	if(Kk>0.5)
		act_value=measureData;
	
	return act_value;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
