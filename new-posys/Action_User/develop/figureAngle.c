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
#include "figurePos.h"
#include "math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static float **a_icm;
extern float *oldResult;
extern uint32_t *oldCountNum;
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
float KalmanFilterT(float ordata);
float KalmanFilterZ(float ordata);
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
static three_axis gyr_icm,gyr_act;  //ԭʼ�Ľ��ٶȺʹ����Ľ��ٶ�
static three_axis acc_icm;          //���ٶ���Ϣ
static float   temp_icm;            //�������¶�

void RoughHandle(void)
{
	static int16_t convert;             //�������¶ȶ�Ӧ�������  
	/*���´�����������*/
	icm_update_gyro_rate();
	icm_update_temp();
	icm_update_acc();
	//LIS3MDL_UpdateMag();
	
	/*��ȡ����*/
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	icm_read_accel_acc(&acc_icm);
	
	temp_icm=KalmanFilterT(temp_icm);
	
	a_icm=GetVdoff_icmErrArr();        //��С������ϵĽ��

	/* �¶�ֵת��������������Ѱ���¶���Ư�����Ӧ��ֵ */
	convert=(int16_t)((temp_icm-TempTable_min)*10.0f/1.0f);
	if((convert>=0&&convert<10*(TempTable_max-TempTable_min))
	    &&oldCountNum[convert]>10           //�����ֵ��������Ҫ����10
	    &&oldCountNum[convert]!=0xffffffff) //��������Ҫ��ֵ,ԭ��flashĬ�϶���1
	{
		gyr_act.x=(gyr_icm.x-oldResult[convert]);
		gyr_act.y=(gyr_icm.y-oldResult[convert+10*(TempTable_max-TempTable_min)]);
		gyr_act.z=(gyr_icm.z-oldResult[convert+20*(TempTable_max-TempTable_min)]);
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
}

	//30�浽49.9��   ���½�����Ʈ  30 -- 0  30.1 -- 1
void TemporaryHandle(int start)
{
	uint16_t index=(uint16_t)((temp_icm-30.f)*10.0f/1.0f);
	static float gyr_aver[200]={0};
	static float countTemp[200]={0};
	/* ��Ϣ����Ӧ�������˲����˳����ٶ��е����� */
	gyr_act.z=KalmanFilterZ(gyr_act.z);
	if(start==0){
    if(countTemp[index]>0)
			gyr_aver[index]=gyr_aver[index]*countTemp[index]/(countTemp[index]+1)+gyr_act.z/(countTemp[index]+1);
		else
			gyr_aver[index]=gyr_act.z;
		
			countTemp[index]++;
	}else{
		if(countTemp[index]>=10){
			float proportion=(temp_icm-30.f)*10.0f-index;
			/*�ֶβ�ֵ*/
			gyr_act.z=gyr_act.z-(gyr_aver[index]*(1-proportion)+gyr_aver[index+1]*proportion);
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
	if(!wait_flag&&fabs(gyr_act.z)>0.3)
	{
		wait_flag=1;
	}
		
	/* 
	��ֵ ���ڴ�ֵ����Ϊû��
	*/
	if(fabs(gyr_act.z)<0.05)//��λ ��/s
		gyr_act.z=0;
	if(fabs(gyr_act.x)<10)	
		gyr_act.x=0;
	if(fabs(gyr_act.y)<10)
		gyr_act.y=0;
	
	/* �ǶȻ���ת�� */
	gyr_act.x=(gyr_act.x)/180.0f*PI;
	gyr_act.y=(gyr_act.y)/180.0f*PI;
	gyr_act.z=(gyr_act.z)/180.0f*PI;
	
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

	#ifdef DEBUG_ENABLE
	
	USART_OUT_F(result_angle.z);
	#endif 

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

float KalmanFilterZ(float ordata)
{
	uint8_t i;
	
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_now;        //���ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.01;       //ϵͳ����
//	static double R=0.0002;      //��������     
	static double R=0.012;      //��������             
	
	static float IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
	/* ��������û���У��ֵ������û�����������ʹ��0.012 */
	//R=Get_R_Zaxis();
   
	predict=act_value;
	
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=ordata-predict;
	
	/* ��Ϣ�ķ������ */
	Cr=0;
	for(i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/49.0f;
	
	
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

float KalmanFilterT(float ordata)
{
	uint8_t i;
	
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_now;        //���ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.8;       //ϵͳ����
	static double R=0.973067;      //��������          
	
	static float IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
	/* ��������û���У��ֵ������û�����������ʹ��0.012 */
	R=Get_R_Zaxis();
   
	predict=act_value;
	
	//�൱�����������
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=ordata-predict;
	
	/* ��Ϣ�ķ������ */
	Cr=0;
	for(i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/49.0;		//���������֪���Բ��ԣ���Ҫ��ԭ��ʽ��
	
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
	USART_OUT_F(gyr_act.z);
	USART_OUT_F(result_angle.z);
	USART_OUT(USART1,"\r\n");
	#endif
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
