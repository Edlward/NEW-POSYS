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
#include "leastSquare.h"
#include "stm32f4xx_it.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
extern float *chartWX;
extern float *chartWY;
extern float *chartWZ;
extern uint32_t *chartNum;
static three_axis result_angle={0,0,0};
static Quarternion quaternion={1,0,0,0};
/* Extern   variables ---------------------------------------------------------*/
/*  ȫ�ֱ���  */
float K_acc=1;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
double KalmanFilterZ(double measureData);
double KalmanFilterY(double measureData);
double KalmanFilterX(double measureData);


/**
  * @brief  ���½Ƕ�ֵ
  *
  * @param[in] gyrData,���������ǵļ��ٶ�ֵ
  *            magData,��������Ƶ�ֵ
  *            accData,������ٶȵ�ֵ
  * @retval ��ʼ����ɵı�־λ
  */
	
static three_axis_d gyr_act;			//ԭʼ�Ľ��ٶȺʹ����Ľ��ٶ�
static gyro_t acc_icm;          //���ٶ���Ϣ
static float   temp_icm;            //�������¶�
int chartIndex = 0;
static float gyr_AVER[3][TempTable_Num]={0.0};
void RoughHandle(void)
{
  gyro_t gyr_icm={0.f};
	static float proportion=0.f;
	/*���´�����������*/
	icm_update_gyro_rate();
	icm_update_temp();
	
	/*��ȡ����*/
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	
	static uint32_t ignore=0;
	ignore++;
	if(ignore<50)
		return ;
	else if(ignore==50)
	{
		for(uint32_t i=0;i<TempTable_Num;i++){
			gyr_AVER[0][i]=chartWX[i];
			gyr_AVER[1][i]=chartWY[i];
			gyr_AVER[2][i]=chartWZ[i];
		}
	}else
		ignore=51;
	
	temp_icm=KalmanFilterT(temp_icm);

	/* �¶�ֵת��������������Ѱ���¶���Ư�����Ӧ��ֵ */
	chartIndex=roundf((temp_icm-TempTable_min)*10);
	if((chartIndex>=0&&chartIndex<TempTable_Num)
	    &&chartNum[chartIndex]>=LEASTNUM           //�����ֵ��������Ҫ����LEASTNUM
	    &&chartNum[chartIndex]!=0xffffffff) //��������Ҫ��ֵ,ԭ��flashĬ�϶���1
	{
		  proportion=(temp_icm-TempTable_min)*10-chartIndex;
			/*�ֶβ�ֵ*/
			gyr_act.x=(double)(gyr_icm.No1.x-(gyr_AVER[0][chartIndex]*(1-proportion)+gyr_AVER[0][chartIndex+1]*proportion));
			gyr_act.y=(double)(gyr_icm.No1.y-(gyr_AVER[1][chartIndex]*(1-proportion)+gyr_AVER[1][chartIndex+1]*proportion));
			gyr_act.z=(double)(gyr_icm.No1.z-(gyr_AVER[2][chartIndex]*(1-proportion)+gyr_AVER[2][chartIndex+1]*proportion));
	}
  else /* ���¶ȱ�����Ϣ������ʱ������С���˷���������߲������Ư�Ƶļ���  */
	{
		//gyr_act.z=gyr_icm.No1.z-FitResult(temp_icm);
	}
}
float getTemp_icm(void){
	return temp_icm;
}
static three_axis acc_angle;        //���ٶȼƲ���Ķ�Ӧ�ĽǶ�
	//30�浽49.9��   ���½�����Ʈ  30 -- 0  30.1 -- 1
void TemporaryHandle(void)
{
	static double gyr_aver[3][TempTable_Num]={0.0};
	static double countTemp[TempTable_Num]={0.0};
	static double accInit[3]={0.0,0.0,0.0};
	static uint32_t count=0;
	static uint32_t flag=0;
	
	//����ǰ��ʮ����
	static uint32_t ignore=0;
	if(ignore++<50)
		return ;
	else
		ignore=50;
	
	chartIndex=roundf((temp_icm-TempTable_min)*10);
	if(chartIndex<0||chartIndex>=TempTable_Num) 
		return;
	if(!(GetCommand()&ADJUST)){
    if(countTemp[chartIndex]>0){
			gyr_aver[0][chartIndex]=gyr_aver[0][chartIndex]+gyr_act.x;
			gyr_aver[1][chartIndex]=gyr_aver[1][chartIndex]+gyr_act.y;
			gyr_aver[2][chartIndex]=gyr_aver[2][chartIndex]+gyr_act.z;
		}
		else{
			gyr_aver[0][chartIndex]=gyr_act.x;
			gyr_aver[1][chartIndex]=gyr_act.y;
			gyr_aver[2][chartIndex]=gyr_act.z;
		}
			countTemp[chartIndex]++;
			flag=1;
		
		icm_update_acc();
		icm_read_accel_acc(&acc_icm);
		count++;
		accInit[0]+=acc_icm.No1.x;
		accInit[1]+=acc_icm.No1.y;
		accInit[2]+=acc_icm.No1.z;
	}else if(flag){
			flag=0;
			for(int i=0;i<TempTable_Num;i++){
				if(countTemp[i]!=0){
					for(uint32_t j=0;j<3;j++){
						gyr_aver[j][i]=gyr_aver[j][i]/countTemp[i];
						gyr_AVER[j][i]=gyr_AVER[j][i]+gyr_aver[j][i];
						gyr_aver[j][i]=0.0;
					}
					countTemp[i]=0;
				}
			}
			//SquareFitting(gyr_AVER);
			for(uint32_t i=0;i<3;i++)
				accInit[i]=accInit[i]/count;
			
			/* ��ȡ���ٶȵ�ֵ */
			icm_update_AccRad(accInit,&acc_angle);
			quaternion=Euler_to_Quaternion(acc_angle);
		}
}
uint8_t updateAngle(void)
{	
	static three_axis euler;            //ŷ����
	static three_axis result;									//���սǶȵĻ��Ƚ�
	
	/* ��Ϣ����Ӧ�������˲����˳����ٶ��е����� */
	gyr_act.x=KalmanFilterX(gyr_act.x);
	gyr_act.y=KalmanFilterY(gyr_act.y);
	gyr_act.z=KalmanFilterZ(gyr_act.z);
		
	/* 
	��ֵ ���ڴ�ֵ����Ϊû��
	*/
//	if(fabs(gyr_act.z)<0.05)//��λ ��/s
//		gyr_act.z=0;
	
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
	
//	/* �ںϼ��ٶȼƺ������� */
//	result.x=acc_angle.x;
//	result.y=acc_angle.y;
//	result.z=euler.z;
//	/* �ص���Ԫ�� */
//	quaternion=Euler_to_Quaternion(result);
	
//	/*���ȽǶ�ת�� */
//	result_angle.x= result.x/PI*180.0f;
//	result_angle.y= result.y/PI*180.0f;
	result_angle.z=-euler.z/PI*180.0f;
	
	USART_OUT_F(gyr_act.z);
	USART_OUT_F(result_angle.z);
	USART_OUT(USART1,"\r\n");

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
/*�㷨�� H,��,����Ϊһ*/
double KalmanFilterZ(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.006708655;       //ϵͳ����         
	static double R=0.006708655;      //�������� 
	static double IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
	predict=act_value;
	
	/* ��Ϣ�ķ������ */
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;

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
/*�㷨�� H,��,����Ϊһ*/
double KalmanFilterX(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.011205;       //ϵͳ����         
	static double R=0.011205;      //�������� 
	static double IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
	predict=act_value;
	
	/* ��Ϣ�ķ������ */
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;

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

/*�㷨�� H,��,����Ϊһ*/
double KalmanFilterY(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.01010547;       //ϵͳ����         
	static double R=0.01010547;      //�������� 
	static double IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
	predict=act_value;
	
	/* ��Ϣ�ķ������ */
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;

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


float KalmanFilterT(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
	static double P_last=0.1;   //��һ�ε�Ԥ�����
	static double P_mid;        //��Ԥ������Ԥ��
	static double Kk;           //�˲�����ϵ��
	
	static double Q=0.025;       //ϵͳ����
	static double R=0.025;      //��������         
	
	static double IAE_st[50];    //��¼����Ϣ
	double Cr=0;                //��Ϣ�ķ���
	
	/* ��������û���У��ֵ������û�����������ʹ��0.012 */
	//R=Get_R_Zaxis();
   
	predict=act_value;
	
	//�൱�����������
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;
	
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
	
	return (float)act_value;
}




/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
