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
extern double *chartWX;
extern double *chartWY;
extern double *chartWZ;
extern uint8_t 	*chartMode;
extern uint8_t 	*chartSelect;
extern float  		*minValue;
extern float     *varXYZ;
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
int JudgeAcc(void);

/**
* @brief  ���½Ƕ�ֵ
*
* @param[in] gyrData,���������ǵļ��ٶ�ֵ
*            magData,��������Ƶ�ֵ
*            accData,������ٶȵ�ֵ
* @retval ��ʼ����ɵı�־λ
*/

static three_axis_d gyr_act;			//ԭʼ�Ľ��ٶȺʹ����Ľ��ٶ�
static float gyr_AVER[3]={0.0};
static three_axis acc_angle;        //���ٶȼƲ���Ķ�Ӧ�ĽǶ�
extern gyro_t gyr_data,acc_data;
extern float  temp_icm;
static uint32_t count=0;
static float acc_sum=0.f;
static double driftCoffecient[3]={0.0};
uint8_t sendPermit=1;
int RoughHandle(void)
{
  double drift[3]={0.0,0.0,0.0};
  
  drift[0]=driftCoffecient[0]*(temp_icm/100.f);
  drift[1]=driftCoffecient[1]*(temp_icm/100.f);
  drift[2]=driftCoffecient[2]*(temp_icm/100.f);
  
//  gyr_act.x=(double)gyr_data.No1.x-drift[0];
//  gyr_act.y=(double)gyr_data.No1.y-drift[1];
//  gyr_act.z=(double)gyr_data.No1.z-drift[2];
	
	
  gyr_act.x=KalmanFilterX(gyr_data.No1.x);
  gyr_act.y=KalmanFilterY(gyr_data.No1.y);
  gyr_act.z=KalmanFilterZ(gyr_data.No1.z);
	
  count++;
  if(count==(25*200+2)){
    count--;
    gyr_act.x=(double)(gyr_act.x-gyr_AVER[0]);
    gyr_act.y=(double)(gyr_act.y-gyr_AVER[1]);
    gyr_act.z=(double)(gyr_act.z-gyr_AVER[2]);
		
		#ifdef TEST_SUMMER
//		if(sendPermit){

//		}
		#endif
    return 1;
  }
  
  return 0;
}

//30�浽49.9��   ���½�����Ʈ  30 -- 0  30.1 -- 1
void TemporaryHandle(void)
{
  static double accInit[3]={0.0,0.0,0.0};
  if(count>=20*200&&count<25*200){
    gyr_AVER[0]=gyr_AVER[0]+gyr_act.x;
    gyr_AVER[1]=gyr_AVER[1]+gyr_act.y;
    gyr_AVER[2]=gyr_AVER[2]+gyr_act.z;
    accInit[0]=accInit[0]+acc_data.No1.x;
    accInit[1]=accInit[1]+acc_data.No1.y;
    accInit[2]=accInit[2]+acc_data.No1.z;
  }
  else if(count==25*200){
    count++;
    gyr_AVER[0]=gyr_AVER[0]/(5.f*200.f);
    gyr_AVER[1]=gyr_AVER[1]/(5.f*200.f);
    gyr_AVER[2]=gyr_AVER[2]/(5.f*200.f);
    accInit[0]=accInit[0]/(5.f*200.f);
    accInit[1]=accInit[1]/(5.f*200.f);
    accInit[2]=accInit[2]/(5.f*200.f);
    acc_sum=sqrt(accInit[0]*accInit[0]+accInit[1]*accInit[1]+accInit[2]*accInit[2]);
    /* ��ȡ���ٶȵ�ֵ */
    //icm_update_AccRad(accInit,&acc_angle);
   //quaternion=Euler_to_Quaternion(acc_angle);
  }
}

void updateAngle(void)
{	
  static three_axis_d euler;            //ŷ����
	float maxStaticValue=*minValue;
	
  if((GetCommand()&STATIC)&&(gyr_act.z<0.2f)){
		maxStaticValue=0.15f;
	}
	
//  if(fabs(gyr_act.x)<maxStaticValue)//��λ ��/s
//    gyr_act.x=0.f;	
//  if(fabs(gyr_act.y)<maxStaticValue)//��λ ��/s
//    gyr_act.y=0.f;	
//  if(fabs(gyr_act.z)<maxStaticValue)//��λ ��/s
//    gyr_act.z=0.f;
	
  /*���ٶȻ��ֳ���Ԫ��*/
//  quaternion=QuaternionInt(quaternion,gyr_act);
// // quaternion=QuaternionInt1(quaternion,gyr_act);
//  /* ��Ԫ��ת����ŷ���� */
//  euler=Quaternion_to_Euler(quaternion);

//  if(JudgeAcc()){
//    euler.x=acc_angle.x;
//		euler.y=acc_angle.y;
//    quaternion=Euler_to_Quaternion(euler);
//  }
	static int timepp=0;
	timepp++;
	if(timepp<60*200*60)
		euler.z=euler.z+gyr_act.z*0.005;
	else
		;
	if(euler.z>180.0f)
		euler.z-=360.0f;
	else if(euler.z<-180.0f)
		euler.z+=360.0f;
	USART_OUT_F(gyr_act.z);
	USART_OUT_F(euler.z);
	
//				USART_OUT_F(euler.z);
//				USART_OUT_F(temp_icm);
//				USART_Enter();
  /*���ȽǶ�ת�� */
//  result_angle.x= euler.x/PI*180.0f;
//  result_angle.y= euler.y/PI*180.0f;
		result_angle.z=-euler.z;
}
void SetAngle(float angle){
	three_axis euler;
	euler.x=0.0f;
	euler.y=0.0f;
	euler.z=angle/180.f*PI;
	quaternion=Euler_to_Quaternion(euler);
}
float getActIcm(void){
	return (float)gyr_act.z;
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

int JudgeAcc(void)
{
  float sum=sqrt(acc_data.No1.x*acc_data.No1.x+acc_data.No1.y*acc_data.No1.y+acc_data.No1.z*acc_data.No1.z);

  float X_G,Y_G,Z_G;
  if(sum!=0.0f){
		X_G=(acc_data).No1.x/sum;
		Y_G=(acc_data).No1.y/sum;
		Z_G=(acc_data).No1.z/sum;
	}
  /*��ʼ����Ϊ0,0,g,Ȼ�����ͨ������任��ʽ�����Ƶ�*/
	acc_angle.y= safe_atan2( X_G , -Z_G);
  acc_angle.x=-safe_atan2( Y_G , X_G/sin(acc_angle.y));
	
  if(fabs(sum-acc_sum)<0.001)
    return 1;
  else
    return 0;
}	
//����ֵ
const double stdCoffeicent[3]={ 0.973632,0.513072, 0.2672 };
void driftCoffecientInit(void){
	int selectCount=0;
	switch(*chartMode){
		//���֮ǰ��õı�׼����
		case 0:
			selectCount++;
			driftCoffecient[0]=stdCoffeicent[0];
			driftCoffecient[1]=stdCoffeicent[1];
			driftCoffecient[2]=stdCoffeicent[2];
			break;
		//�����֮ǰ��õı�׼����
		case 1:
			
			break;
	}

	for(int i=0;i<5;i++)
	{
		if(chartSelect[i]){
			selectCount++;
			driftCoffecient[0]=driftCoffecient[0]+chartWX[i];
			driftCoffecient[1]=driftCoffecient[1]+chartWY[i];
			driftCoffecient[2]=driftCoffecient[2]+chartWZ[i];
		}
	}	
	driftCoffecient[0]=driftCoffecient[0]/selectCount;
	driftCoffecient[1]=driftCoffecient[1]/selectCount;
	driftCoffecient[2]=driftCoffecient[2]/selectCount;
	
#ifdef TEST_SUMMER
//	USART_OUT_F(driftCoffecient[0]);
//	USART_OUT_F(driftCoffecient[1]);
//	USART_OUT_F(driftCoffecient[2]);
//	USART_Enter();
#endif
	
}
float safe_asin(float v)
{
 
if (isnan(v)) {
 
	//Ϊʲôһֱ�ڽ���
//	USART_OUT(USART6,(uint8_t*)"zhang1\r\n");
return 0.0f;
 
}
if (v >= 1.0f) {
 
//	USART_OUT(USART6,"2\r\n");
return 3.1415926/2;
 
}
if (v <= -1.0f) {
 
//	USART_OUT(USART6,"zhang3\r\n");
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
//	USART_OUT(USART6,"summer1\r\n");
   return 0.0f;
  }
	
	if(isnan(x/y))
	{
		if(x>0){
//	USART_OUT(USART6,"summer2\r\n");
		  return  3.1415926/2.0; 
		}
		
		else if(x<0){
//	USART_OUT(USART6,"summer3\r\n");
			return -3.1415926/2.0;
		}
			
		else {
//	USART_OUT(USART6,"summer4\r\n");
			return 0.0;
		}
	}
	
	return atan2(x,y);
}
/*�㷨�� H,��,����Ϊһ*/
double KalmanFilterZ(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
  static double P_last=0.01;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.003;       //ϵͳ����        
  double R=0.003;      //�������� 
  static double IAE_st[50];    //��¼����Ϣ
  static double data=0.0;
  double Cr=0;                //��Ϣ�ķ���
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
  predict=act_value;
  
  /* ��Ϣ�ķ������ */
  memcpy(IAE_st,IAE_st+1,392);
  IAE_st[49]=measureData-predict;
  
  Cr=0;
  for(int i=0;i<50;i++)
  {
    Cr=Cr+IAE_st[i]*IAE_st[i];
  }
  Cr=Cr/50.0f;
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return 0.0;
  }
  else if(ignore==100){
    predict=data/99.0;
    //USART_OUT_F(predict);
    //USART_Enter();
  }else
    ignore=101;
  
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
  
  static double P_last=0.0000003169;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.00000000002514;       //ϵͳ����         
  double R=(double)varXYZ[0];      //�������� 
  static double IAE_st[50];    //��¼����Ϣ
  static double data=0.0;
  double Cr=0;                //��Ϣ�ķ���
  
  predict=act_value;
  
  /* ��Ϣ�ķ������ */
  memcpy(IAE_st,IAE_st+1,392);
  IAE_st[49]=measureData-predict;
  
  Cr=0;
  for(int i=0;i<50;i++)
  {
    Cr=Cr+IAE_st[i]*IAE_st[i];
  }
  Cr=Cr/50.0f;
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return 0.0;
  }
  else if(ignore==100){
    predict=data/99.0;
    //USART_OUT_F(predict);
    //USART_Enter();
  }else
    ignore=101;
  
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
  
  static double P_last=0.0000003169;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.00000000002514;       //ϵͳ����         
  double R=(double)varXYZ[1];      //�������� 
  static double IAE_st[50];    //��¼����Ϣ
  static double data=0.0;
  double Cr=0;                //��Ϣ�ķ���
  
  predict=act_value;
  
  /* ��Ϣ�ķ������ */
  memcpy(IAE_st,IAE_st+1,392);
  IAE_st[49]=measureData-predict;
  
  Cr=0;
  for(int i=0;i<50;i++)
  {
    Cr=Cr+IAE_st[i]*IAE_st[i];
  }
  Cr=Cr/50.0f;
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return 0.0;
  }
  else if(ignore==100){
    predict=data/99.0;
    //USART_OUT_F(predict);
    //USART_Enter();
  }else
    ignore=101;
  
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
  
  static double P_last=0.00001505;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.000000009957;       //ϵͳ����
  static double R=0.01286;      //��������         
  
  static double IAE_st[50];    //��¼����Ϣ
  double Cr=0;                //��Ϣ�ķ���
  static double data=0.0;
  
  predict=act_value;
  
  /* ��Ϣ�ķ������ */
  memcpy(IAE_st,IAE_st+1,392);
  IAE_st[49]=measureData-predict;
  
  Cr=0;
  for(int i=0;i<50;i++)
  {
    Cr=Cr+IAE_st[i]*IAE_st[i];
  }
  Cr=Cr/50.0f;
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return 0.0;
  }
  else if(ignore==100){
    predict=data/99.0;
    //USART_OUT_F(predict);
    //	USART_Enter();
  }else
    ignore=101;
  
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
