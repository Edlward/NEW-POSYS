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
#include "motion_attitude_algorithm.h"
#include "string.h"
#include "usart.h"
#include "timer.h"
#include "flash.h"
#include "temperature_control.h"
#include "vdoff.h"
#include "customer.h"
#include "pos.h"
#include "math.h"
#include "action_AHRS.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static void statistic(float w);
static float *VDoff;
static float **a_icm;
static uint32_t *countnum_icm;
static three_axis result_angle={0,0,0};
static Quarternion quaternion={1,0,0,0};
/* Extern   variables ---------------------------------------------------------*/
/*  ȫ�ֱ���  */
float K_acc=1;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
static Quarternion QuaternionInt(Quarternion quaternion,three_axis data);
static three_axis Quaternion_to_Euler(Quarternion quaternion);
static Quarternion Euler_to_Quaternion(three_axis Rad);
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
uint8_t updateAngle(void)
{	
	static three_axis euler;            //ŷ����
	static three_axis gyr_icm,gyr_act;  //ԭʼ�Ľ��ٶȺʹ����Ľ��ٶ�
	static three_axis acc_icm;          //���ٶ���Ϣ
	
	static float   temp_icm;            //�������¶�
	static int16_t convert;             //�������¶ȶ�Ӧ�������  
	
	static three_axis acc_angle;        //���ٶȼƲ���Ķ�Ӧ�ĽǶ�
	
	static uint8_t wait_flag=0;
	static uint8_t wait_second=0;
	
	
	three_axis result;									//���սǶȵĻ��Ƚ�
	#ifdef DEBUG_ENABLE
	/*
	�ô��ڷ�����Ϣ����debug
	*/
	  static int timede=0;
	#endif
	
	/*���´�����������*/
	icm_update_gyro_rate();
	icm_update_temp();
	icm_update_acc();
	//LIS3MDL_UpdateMag();
	
	/*��ȡ����*/
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	icm_read_accel_acc(&acc_icm);
	//LIS_readMag(&mag);
	/*��ȡ��Ư��Ϣ*/
	VDoff=GetVdoff_icmArr();           //�¶���Ư���е���Ϣ
	countnum_icm=GetCountnum_icmArr(); //��ȡ����ֵ
	a_icm=GetVdoff_icmErrArr();        //��С������ϵĽ��

	/* �¶�ֵת��������������Ѱ���¶���Ư�����Ӧ��ֵ */
	convert=(int16_t)((temp_icm-TempTable_min)*10.0f/1.0f);
	if((convert>=0&&convert<10*(TempTable_max-TempTable_min))
	    &&countnum_icm[convert]>10           //�����ֵ��������Ҫ����10
	    &&countnum_icm[convert]!=0xffffffff) //��������Ҫ��ֵ,ԭ��flashĬ�϶���1
	{
		gyr_act.x=(gyr_icm.x-VDoff[convert]);
		gyr_act.y=(gyr_icm.y-VDoff[convert+10*(TempTable_max-TempTable_min)]);
		gyr_act.z=(gyr_icm.z-VDoff[convert+20*(TempTable_max-TempTable_min)]);
	}
  else /* ���¶ȱ�����Ϣ������ʱ������С���˷���������߲������Ư�Ƶļ���  */
	{
		gyr_act.x=gyr_icm.x;
		gyr_act.y=gyr_icm.y;
		gyr_act.z=gyr_icm.z;
		/*��ȥһ��һ�κ���*/
		for(uint8_t i=0;i<BF_TH;i++)
		{
			gyr_act.x=gyr_act.x-a_icm[i][0]*pow(temp_icm,i);
			gyr_act.y=gyr_act.y-a_icm[i][1]*pow(temp_icm,i);
			gyr_act.z=gyr_act.z-a_icm[i][2]*pow(temp_icm,i);
		}
	}
	
	statistic(gyr_act.z);
	
	/* ���ν������� */
	/*
	Ϊ�˼�ȥ��Ʈ������Ӱ��
	��ʼ��ʱȡһ������ݽ�����ƽ��ֵ
	Ȼ���˶�������ʱ�̼�ȥ
	*/
	wait_second=adjustVDoff(&gyr_act);

	#ifdef DEBUG_ENABLE
		timede++;
		if(timede>300)
		{
			timede=0;
		}
		if(timede%20==0)
		{
			USART_OUT(USART1,"\r\ntemp:%d\r\n",(int)(temp_icm*100));
		}
		if(timede%20==4)
		{
			USART_OUT(USART1,"angle:%d\r\n",(int)(result_angle.z*100));
		}
		if(timede%20==8)
		{
			USART_OUT(USART1,"wz:%d\r\n",(int)(gyr_act.z*10000));
		}
		if(timede%20==12)
		{
			USART_OUT(USART1,"x:%d\r\n",(int)(getPosX()));
		}
		if(timede%20==16)
		{
			USART_OUT(USART1,"y:%d\r\n",(int)(getPosY()));
		}
	#endif
		
	/* ��Ϣ����Ӧ�������˲����˳����ٶ��е����� */
	gyr_act.z=KalmanFilterZAxis(gyr_act.z);
	gyr_act.x=KalmanFilterXAxis(gyr_act.x);
	gyr_act.y=KalmanFilterYAxis(gyr_act.y);
	
	
	/* �ڳ���֮ǰ�����л��ֵı�־λ */
	if(!wait_flag&&fabs(gyr_act.z)>0.3)
	{
		wait_flag=1;
	}
		
	/* 
	��ֵ ���ڴ�ֵ����Ϊû��
	*/
	if(fabs(gyr_act.z)<0.15)//��λ ��/s
		gyr_act.z=0;
	if(fabs(gyr_act.x)<10)	
		gyr_act.x=0;
	if(fabs(gyr_act.y)<10)
		gyr_act.y=0;
	/*��ȡz���ٶȵĽӿ�*/
	/* �ǶȻ���ת�� */
	gyr_act.x=(gyr_act.x)/180.0f*PI;
	gyr_act.y=(gyr_act.y)/180.0f*PI;
	gyr_act.z=(gyr_act.z)/180.0f*PI;
	
	
//	quaternion=AHRS_Update(gyr_act,acc_icm,mag);
	
	/* ��Ԫ������ */
	if(wait_flag&&(gyr_act.x!=0||
		             gyr_act.y!=0||
	               gyr_act.z!=0))
	/*���ٶȻ��ֳ���Ԫ��*/
	/*
	ѡ��ZXY��ת˳��,��Ϊ�����������ǶԺ���Ǻ͸����ǵ���ʶ
	��ν�Ժ���˳����ת,�������������.
	һ�Ǵ�˳����תŷ������������Ԫ��,��ν��ת˳��ͬ,�����̬��ͬ,��Ԫ��Ҳ��ͬ
	���Ǵ��Թ̶����ٶȰ���Ԫ�����ֳ�һ���̶���̬,���Ƶ���ŷ����,�Ժ���˳����ȫȡ�������Լ�����ʶ
	x����90,yz����180.
	*/
	  quaternion=QuaternionInt(quaternion,gyr_act);
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
			/*���ȽǶ�ת�� */
	acc_angle.x= acc_angle.x/PI*180.0f;
	acc_angle.y= acc_angle.y/PI*180.0f;
	return wait_second;
}
/**
  * @brief  ���ν������Ư��
  * @param  w:  ����Ľ��ٶ�
  * @retval ������Ľ��ٶ�
  */
uint8_t adjustVDoff(three_axis *w)
{
	/* ʱ�����ֵ */
	/*count��������*/
	static uint32_t count=0;
	static three_axis adjust={0,0,0};
	
	/* ʱ����ƣ���������Ϊ200Hz��200��1�� */
	if(count<=200)
	{
	 count++;
	 adjust.x=adjust.x+w->x;
	 adjust.y=adjust.y+w->y;
	 adjust.z=adjust.z+w->z;
		
	 /* �������ǰ��������Ľ��ٶ�Ϊ0 */
	 w->x=0;
	 w->y=0;
	 w->z=0;
		
	 return 0;
	}
	else
	{
		/* �õ�������Ľ��ٶ� */
		w->x=w->x-adjust.x/200.0f;
		w->y=w->y-adjust.y/200.0f;
		w->z=w->z-adjust.z/200.0f;
	}
	return 1;
}


/**
  * @brief  ����ͳ��
  * @param  w: ������ٶ�
  * @retval none
  */
static void statistic(float w)
{
	static float w_old[50];
	static float w_aver;
	static float w_R;
	
	memcpy(w_old,w_old+1,49*4);//??49����
	w_old[49]=w;
	
	w_aver=0;
	for(uint8_t i=0;i<50;i++)
	{
		w_aver+=w_old[i];
	}
	/*����ֵС��0.15*/
	w_aver/=50;
	
	w_R=0;
	for(uint8_t i=0;i<50;i++)
	{
		w_R+=(w_old[i]-w_aver)*(w_old[i]-w_aver);
	}
	/*����ֵӦС��0.0012*/
	w_R/=49;
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
/**
  * @brief  ����Ԫ��ת��Ϊŷ����
  * @param  quaternion: ��Ҫת������Ԫ��
  * @retval ��Ԫ����Ӧ��ŷ����
  */
static three_axis Quaternion_to_Euler(Quarternion quaternion)
{
	three_axis Rad;
	float q0,q1,q2,q3;
	float sum;
		
	q0=quaternion.q0;
	q1=quaternion.q1;
	q2=quaternion.q2;
	q3=quaternion.q3;
	
	sum=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0/sum;
	q1=q1/sum;
	q2=q2/sum;
	q3=q3/sum;
	
	Rad.x= safe_asin(2.0f*(q2*q3 + q0*q1));
	Rad.y= atan2(-2 * q1 * q3 + 2 * q0 * q2,  q3*q3 - q2 * q2 - q1 * q1 +q0 * q0);
	Rad.z= atan2(2*q1*q2-2*q0*q3,q2*q2-q3*q3+q0*q0-q1*q1);
	return Rad;
}

/**
  * @brief  ��ŷ����ת��Ϊ��Ԫ��
  * @param  quaternion: ��Ҫת����ŷ����
  * @retval ��Ԫ����Ӧ����Ԫ��
  */
static Quarternion Euler_to_Quaternion(three_axis Rad)
{
  Quarternion quaternion;

	quaternion.q0=0.5*sqrt(1+cos(Rad.y)*cos(Rad.z)+cos(Rad.x)*cos(Rad.z)+cos(Rad.x)*cos(Rad.y)+sin(Rad.x)*sin(Rad.y)*sin(Rad.z));
	if(quaternion.q0==0)
	{
		quaternion.q0=1;
		quaternion.q1=0;
		quaternion.q2=0;
		quaternion.q3=0;
	}
	else
	{
	  quaternion.q1=(sin(Rad.x)+sin(Rad.y)*sin(Rad.z)+sin(Rad.x)*cos(Rad.y)*cos(Rad.z))/4/quaternion.q0;
	  quaternion.q2=(sin(Rad.y)*cos(Rad.z)-cos(Rad.y)*sin(Rad.z)*sin(Rad.x)+sin(Rad.y)*cos(Rad.x))/4/quaternion.q0;
	  quaternion.q3=(-cos(Rad.y)*sin(Rad.z)+sin(Rad.y)*cos(Rad.z)*sin(Rad.x)-sin(Rad.z)*cos(Rad.x))/4/quaternion.q0;
	}
	
	return quaternion; 
}
/**
  * @brief  �����Ԫ������ٶȵĻ���  �������������
  * @param  quaternion: ԭʼ����̬
  * @param  data      : �豸�Ľ��ٶ�
  * @retval ����������̬
  */
static Quarternion QuaternionInt(Quarternion quaternion,three_axis data)
{          
	static three_axis old_w={0,0,0};
  Quarternion dif_quarterion_f;
	Quarternion dif_quarterion_l;
	Quarternion med_quarterion;
	
	dif_quarterion_f.q0=(-quaternion.q1*old_w.x - quaternion.q2*old_w.y - quaternion.q3*old_w.z)*0.5f;
	dif_quarterion_f.q1=( quaternion.q0*old_w.x + quaternion.q2*old_w.z - quaternion.q3*old_w.y)*0.5f;
	dif_quarterion_f.q2=( quaternion.q0*old_w.y - quaternion.q1*old_w.z + quaternion.q3*old_w.x)*0.5f;
	dif_quarterion_f.q3=( quaternion.q0*old_w.z + quaternion.q1*old_w.y - quaternion.q2*old_w.x)*0.5f;
	/*#define dT 					   0.005f           //���ֵĲ���5ms*/
	med_quarterion.q0=quaternion.q0+dif_quarterion_f.q0*dT;
	med_quarterion.q1=quaternion.q1+dif_quarterion_f.q1*dT;
	med_quarterion.q2=quaternion.q2+dif_quarterion_f.q2*dT;
	med_quarterion.q3=quaternion.q3+dif_quarterion_f.q3*dT; 
  
	dif_quarterion_l.q0=(-med_quarterion.q1*data.x - med_quarterion.q2*data.y - med_quarterion.q3*data.z)*0.5f;
	dif_quarterion_l.q1=( med_quarterion.q0*data.x + med_quarterion.q2*data.z - med_quarterion.q3*data.y)*0.5f;
	dif_quarterion_l.q2=( med_quarterion.q0*data.y - med_quarterion.q1*data.z + med_quarterion.q3*data.x)*0.5f;
	dif_quarterion_l.q3=( med_quarterion.q0*data.z + med_quarterion.q1*data.y - med_quarterion.q2*data.x)*0.5f;
	
	
	quaternion.q0=quaternion.q0+0.5f*(dif_quarterion_f.q0+dif_quarterion_l.q0)*dT;
	quaternion.q1=quaternion.q1+0.5f*(dif_quarterion_f.q1+dif_quarterion_l.q1)*dT;
	quaternion.q2=quaternion.q2+0.5f*(dif_quarterion_f.q2+dif_quarterion_l.q2)*dT;
	quaternion.q3=quaternion.q3+0.5f*(dif_quarterion_f.q3+dif_quarterion_l.q3)*dT;
	
	
	old_w=data;
	return quaternion;
}

float KalmanFilterZAxis(float ordata)
{
	uint8_t i;
	
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_now;        //���ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.01;       //ϵͳ����
	static double R=0.007;      //��������          
	
	static float IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
	/* ��������û���У��ֵ������û�����������ʹ��0.012 */
	R=Get_R_Zaxis();
   
	predict=act_value;
	
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=ordata-predict;
	
	/* ��Ϣ�ķ������ */
	Cr=0;
	for(i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/50.0;
	
	
	/* Ԥ�Ȿ�ε�Ԥ����� */
	P_mid=P_last+Q;
	
	/* ����ϵ����������ֵ */
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(ordata-predict);
	
	/* ����Ԥ����� */
	P_now=(1-Kk)*P_mid;
	
	/* ���㲢����ϵͳ���� */
	Q=Kk*Kk*Cr;
	
	P_last=P_now;

	/* Ϊ����˲�������Ӧ�ٶȣ���С�ͺ�����µ���ֵ */
	if(Kk>0.5)
		act_value=ordata;
	
	return act_value;
}
float KalmanFilterXAxis(float ordata)
{
	uint8_t i;
	
  static double act_value=0;
  double predict;
  
	static double P_last=0.1;
	static double P_now;
	static double P_mid;
	static double Kk;
	
	static double Q=0.01;
	static double R=0.01;
	
	static float IAE_st[50];
	double Cr=0;
	
	predict=act_value;
	
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=ordata-predict;
	
	Cr=0;
	for(i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/50.0;
	
	
	P_mid=P_last+Q;
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(ordata-predict);
	
	P_now=(1-Kk)*P_mid;
	
	//R=Cr-P_mid;
	Q=Kk*Kk*Cr;
	
	P_last=P_now;
	
//	
	if(Kk>0.6)
		act_value=ordata;
	
//	if(fabs(ordata)>0.5)
//	{
//		USART_OUT(USART1,"%d ",(int)(fabs(act_value-ordata)*10000));
//	}
	
	return act_value;
}
float KalmanFilterYAxis(float ordata)
{
	uint8_t i;
	
  static double act_value=0;
  double predict;
  
	static double P_last=0.1;
	static double P_now;
	static double P_mid;
	static double Kk;
	
	static double Q=0.01;
	static double R=0.015;
	
	static float IAE_st[50];
	double Cr=0;
	
	predict=act_value;
	
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=ordata-predict;
	
	Cr=0;
	for(i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/50.0;
	
	
	P_mid=P_last+Q;
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(ordata-predict);
	
	P_now=(1-Kk)*P_mid;
	
	//R=Cr-P_mid;
	Q=Kk*Kk*Cr;
	
	P_last=P_now;
	
//	
	if(Kk>0.6)
		act_value=ordata;
	
//	if(fabs(ordata)>0.5)
//	{
//		USART_OUT(USART1,"%d ",(int)(fabs(act_value-ordata)*10000));
//	}
	
	return act_value;
}
void resetAngle(void)
{
	result_angle.x=0;
	result_angle.y=0;
	result_angle.z=0;
	
	quaternion.q0=1;
	quaternion.q1=0;
	quaternion.q2=0;
	quaternion.q3=0;
}
void setAngle(float zAngle)
{
	result_angle.z=zAngle;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
