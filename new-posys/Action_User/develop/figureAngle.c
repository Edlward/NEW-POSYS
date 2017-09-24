/**
  ******************************************************************************
  * @file     motion_attitude_algorithm.c  
  * @author   Lxy Zlq Action
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
#include "figureAngle.h"
#include "string.h"
#include "usart.h"
#include "timer.h"
#include "flash.h"
#include "temperature_control.h"
#include "buildExcel.h"
#include "customer.h"
#include "arm_math.h"
#include "quarternion.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float **a_icm;
extern float *chartW;
extern uint32_t *chartNum;
static three_axis result_angle={0,0,0};
static Quarternion quaternion={1,0,0,0};
/* Extern   variables ---------------------------------------------------------*/
/*  ȫ�ֱ���  */
float K_acc=1;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
float KalmanFilterT(float measureData);
float KalmanFilterZ(float measureData);
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/



/**
  * @brief  ���½Ƕ�ֵ
  *
  * @param[in] gyrData,���������ǵļ��ٶ�ֵ
  *            magData,��������Ƶ�ֵ
  *            accData,������ٶȵ�ֵ
  * @retval ��ʼ����ɵı�־λ
  */
	
static gyro_t gyr_icm,gyr_act;			//ԭʼ�Ľ��ٶȺʹ����Ľ��ٶ�
static gyro_t acc_icm;          //���ٶ���Ϣ
static float   temp_icm;            //�������¶�
static float   temp_icm_filter;            //�������¶�

void RoughHandle(void)
{
	static int16_t convert;             //�������¶ȶ�Ӧ�������  
	/*���´�����������*/
	icm_update_gyro_rate();
	icm_update_temp();
	icm_update_acc();
	
	/*��ȡ����*/
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	//icm_read_accel_acc(&acc_icm);
	
	temp_icm=KalmanFilterT(temp_icm);
	

	/* �¶�ֵת��������������Ѱ���¶���Ư�����Ӧ��ֵ */
	convert=(int16_t)((temp_icm-TempTable_min)*10.0f/1.0f);
	if((convert>=0&&convert<10*(TempTable_max-TempTable_min))
	    &&chartNum[convert]>10           //�����ֵ��������Ҫ����10
	    &&chartNum[convert]!=0xffffffff) //��������Ҫ��ֵ,ԭ��flashĬ�϶���1
	{
		gyr_act.No1.x=(gyr_icm.No1.x-chartW[convert]);
		gyr_act.No1.y=(gyr_icm.No1.y-chartW[convert+10*(TempTable_max-TempTable_min)]);
		gyr_act.No1.z=(gyr_icm.No1.z-chartW[convert+20*(TempTable_max-TempTable_min)]);
	}
  else /* ���¶ȱ�����Ϣ������ʱ������С���˷���������߲������Ư�Ƶļ���  */
	{
		gyr_act.No1.x=gyr_icm.No1.x;
		gyr_act.No1.y=gyr_icm.No1.y;
		gyr_act.No1.z=gyr_icm.No1.z;
		/*��ȥһ��һ�κ���*//*ǰ����ִ����Ϻ�����������߻�Խ�����*/
		for(uint8_t i=0;i<BF_TH;i++)
		{
			gyr_act.No1.x=gyr_act.No1.x-a_icm[i][0]*pow(temp_icm,i);
			gyr_act.No1.y=gyr_act.No1.y-a_icm[i][1]*pow(temp_icm,i);
			gyr_act.No1.z=gyr_act.No1.z-a_icm[i][2]*pow(temp_icm,i);
		}
	}
}

	//30�浽49.9��   ���½�����Ʈ  30 -- 0  30.1 -- 1
void TemporaryHandle(int start)
{
	uint32_t index=(uint32_t)((temp_icm-30.f)*10.0f/1.0f);
	static float gyr_aver[200]={0};
	static float countTemp[200]={0};
	/* ��Ϣ����Ӧ�������˲����˳����ٶ��е����� */
	gyr_act.No1.z=KalmanFilterZ(gyr_act.No1.z);
	if(start==0){
    if(countTemp[index]>0)
			gyr_aver[index]=gyr_aver[index]*countTemp[index]/(countTemp[index]+1)+gyr_act.No1.z/(countTemp[index]+1);
		else
			gyr_aver[index]=gyr_act.No1.z;
		
			countTemp[index]++;
	}else{
		if(countTemp[index]>=10){
			float proportion=(temp_icm-30.f)*10.0f-index;
			/*�ֶβ�ֵ*/
			gyr_act.No1.z=gyr_act.No1.z-(gyr_aver[index]*(1-proportion)+gyr_aver[index+1]*proportion);
		}
	}
}
	
static three_axis acc_angle;        //���ٶȼƲ���Ķ�Ӧ�ĽǶ�
uint8_t updateAngle(void)
{	
	static three_axis euler;            //ŷ����
	static uint8_t wait_flag=0;
	static uint8_t wait_second=0;
	static three_axis result;									//���սǶȵĻ��Ƚ�
	
	/* ���ν������� */
	/*
	Ϊ�˼�ȥ��Ʈ������Ӱ��
	��ʼ��ʱȡһ������ݽ�����ƽ��ֵ
	Ȼ���˶�������ʱ�̼�ȥ
	*/
	
	/* �ڳ���֮ǰ�����л��ֵı�־λ */
	if(!wait_flag&&fabs(gyr_act.No1.z)>0.3)
	{
		wait_flag=1;
	}
		
	/* 
	��ֵ ���ڴ�ֵ����Ϊû��
	*/
	if(fabs(gyr_act.No1.z)<0.05)//��λ ��/s
		gyr_act.No1.z=0;
	if(fabs(gyr_act.No1.x)<10)	
		gyr_act.No1.x=0;
	if(fabs(gyr_act.No1.y)<10)
		gyr_act.No1.y=0;
	
	/* �ǶȻ���ת�� */
	gyr_act.No1.x=(gyr_act.No1.x)/180.0f*PI;
	gyr_act.No1.y=(gyr_act.No1.y)/180.0f*PI;
	gyr_act.No1.z=(gyr_act.No1.z)/180.0f*PI;
	
	/*���ٶȻ��ֳ���Ԫ��*/
	/*
	ѡ��ZXY��ת˳��,��Ϊ�����������ǶԺ���Ǻ͸����ǵ���ʶ
	��ν�Ժ���˳����ת,�������������.
	һ�Ǵ�˳����תŷ������������Ԫ��,��ν��ת˳��ͬ,�����̬��ͬ,��Ԫ��Ҳ��ͬ
	���Ǵ��Թ̶����ٶȰ���Ԫ�����ֳ�һ���̶���̬,���Ƶ���ŷ����,�Ժ���˳����ȫȡ�������Լ�����ʶ
	x����90,yz����180.
	*/
	 quaternion=QuaternionInt(quaternion,gyr_act.No1);
	/*
	���������໥��ϵ�,�ܹ�����Ӱ��
	*/
  /* ��Ԫ��ת����ŷ���� */	
	euler=Quaternion_to_Euler(quaternion);
	
	/* ��ȡ���ٶȵ�ֵ */
	icm_update_AccRad(&acc_angle);
	
	/* �ںϼ��ٶȼƺ������� */
	result.x=euler.x*K_acc+acc_angle.x*(1-K_acc);
	result.y=euler.y*K_acc+acc_angle.y*(1-K_acc);
	result.z=euler.z;
	/* �ص���Ԫ�� */
	quaternion=Euler_to_Quaternion(result);
	
	/*���ȽǶ�ת�� */
	result_angle.x= result.x/PI*180.0f;
	result_angle.y= result.y/PI*180.0f;
	result_angle.z=-result.z/PI*180.0f;

	return wait_second;
}


void Test(int finish){
	
	static float gyr_aver_rough[2000]={0};
	static float gyr_aver[2000]={0};
	static float countTemp[2000]={0};
	static float countTemp_rough[2000]={0};
	static uint32_t index=0;
	static uint32_t index_rough=0;
	static uint32_t count=0;
	static int ii=0;
	
	if(finish==1&&ii<2000){
//			USART_OUT_F(gyr_aver_rough[ii]);
//			USART_OUT_F(countTemp_rough[ii]);
//			USART_OUT_F(gyr_aver[ii]);
//			USART_OUT_F(countTemp[ii]);
//			USART_OUT(USART6,"\r\n");
//			ii++;
			return ;
	}
	
	icm_update_gyro_rate();
	icm_update_temp();
	
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	
	USART_OUT_F(gyr_icm.No1.z);
	USART_OUT_F(temp_icm);
	gyr_act.No1.z=KalmanFilterZ(gyr_icm.No1.z);
	temp_icm_filter=KalmanFilterT(temp_icm);
	
	gyr_act.No1.z=KalmanFilterZ(gyr_icm.No1.z);
	temp_icm_filter=KalmanFilterT(temp_icm);
	
	if(temp_icm_filter>30.f){
		index=(uint32_t)((temp_icm_filter-30.f)*100.0f/1.0f);
		if(countTemp[index]>0){
			gyr_aver[index]=gyr_aver[index]*countTemp[index]/(countTemp[index]+1)+gyr_act.No1.z/(countTemp[index]+1);
		}
		else{
			gyr_aver[index]=gyr_act.No1.z;
		}
			
			countTemp[index]++;
	}
	
	if(temp_icm>30.f){
		index_rough=(uint32_t)((temp_icm-30.f)*100.0f/1.0f);
		if(countTemp_rough[index_rough]>0){
			gyr_aver_rough[index_rough]=gyr_aver_rough[index_rough]*countTemp_rough[index_rough]/(countTemp_rough[index_rough]+1)+gyr_icm.No1.z/(countTemp_rough[index_rough]+1);
		}
		else{
			gyr_aver_rough[index_rough]=gyr_icm.No1.z;
		}
			countTemp_rough[index_rough]++;
	}
	count++;
	if(count==200){
		count=0;
		USART_OUT_F(temp_icm);
		USART_OUT(USART6,"\r\n");
	}
}



/**
  * @brief  �õ�������ĽǶ�
  * @param  none
  * @retval ����ĽǶ�
  */
three_axis getAngle(void)
{
	return result_angle;
}
/**
  * @brief  �Ż���ķ����Ǻ���
  * @param  v: ��Ӧ��sinֵ
  * @retval �õ�v�����ҵ�ֵ
  */
float safe_asin(float v)
{
 
if (isnan(v)) {
 
return 0.0f;
 
}
if (v >= 1.0f) {
 
return 3.1415926/2;
 
}
if (v <= -1.0f) {
 
return -3.1415926/2;
 
}
return asin(v);
 
}
/**
  * @brief  �Ż���ķ����Ǻ���
  * @param  x: tan=x/y
  * @param  y:
  * @retval �õ������е�ֵ
  */
double safe_atan2(double x,double y)
{
	if (isnan(y)) 
	{ 
   return 0.0f;
  }
	
	if(isnan(x/y))
	{
		if(x>0)
		  return  3.1415926/2.0; 
		
		else if(x<0)
			return -3.1415926/2.0;
			
		else 
			return 0.0;
	}
	
	return atan2(x,y);
}
/*�㷨�� H,��,����Ϊһ*/
float KalmanFilterZ(float measureData)
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

float KalmanFilterT(float measureData)
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


//#define ACC_DEBUG
#define ICM_DEBUG
void DebugMode(void)
{
	#ifdef ACC_DEBUG
			/*���ȽǶ�ת�� */
	acc_angle.x= acc_angle.x/PI*180.0f;
	acc_angle.y= acc_angle.y/PI*180.0f;
	
	USART_OUT_F(acc_icm.x);
	USART_OUT_F(acc_angle.x);
	USART_OUT_F(acc_icm.y);
	USART_OUT_F(acc_angle.y);
	USART_OUT(USART1,(uint8_t*)"\r\n");
	#endif
	#ifdef ICM_DEBUG
	//gyr_act.z=gyr_act.z*57.29577f;
	USART_OUT_F(gyr_act.No1.z);
	USART_OUT_F(result_angle.z);
	USART_OUT(USART1,"\r\n");
	#endif
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
