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

#include "config.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

extern flashData_t flashData;
extern AllPara_t allPara;
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

static float gyr_AVER[3]={0.0};
static uint32_t count=0;
uint8_t sendPermit=1;
int RoughHandle(void)
{
  allPara.GYRO_Real[0]=KalmanFilterX(allPara.GYRO_Aver[0]);
  allPara.GYRO_Real[1]=KalmanFilterY(allPara.GYRO_Aver[1]);
  allPara.GYRO_Real[2]=KalmanFilterZ(allPara.GYRO_Aver[2]);
	
//  allPara.GYRO_Real[0]=(allPara.GYRORemoveDrift[0][0]);
//  allPara.GYRO_Real[1]=(allPara.GYRORemoveDrift[0][1]);
//  allPara.GYRO_Real[2]=(allPara.GYRORemoveDrift[0][2]);
	if(count!=15*200)
		count++;
  if(count==(15*200+4)&&(GetCommand()&ACCUMULATE)){
    count--;
    allPara.GYRO_Real[0]=(double)(allPara.GYRO_Real[0]-gyr_AVER[0]);
    allPara.GYRO_Real[1]=(double)(allPara.GYRO_Real[1]-gyr_AVER[1]);
    allPara.GYRO_Real[2]=(double)(allPara.GYRO_Real[2]-gyr_AVER[2]);
		USART_OUT_F(allPara.GYRO_TemperatureDif[0]);
		USART_OUT_F( allPara.GYRO_Real[2]);
		USART_Enter();
    return 1;
  }else{
		return 0;
	}
  
}

//30�浽49.9��   ���½�����Ʈ  30 -- 0  30.1 -- 1
void TemporaryHandle(void)
{
  static float accInit[GYRO_NUMBER][AXIS_NUMBER]={
	2.0,0.7,0.05,
	0.3,0.3,-0.05,
	0.3,0.3,-0.05
	};
	int gyro=0;
	int axis=0;
	
  if(count>=10*200&&count<15*200){
		for(axis=0;axis<AXIS_NUMBER;axis++)
		{
			gyr_AVER[axis]=gyr_AVER[axis]+allPara.GYRO_Real[axis];
			for(gyro=0;gyro<GYRO_NUMBER;gyro++)
			{
				accInit[gyro][axis]=accInit[gyro][axis]+allPara.ACC_Aver[gyro][axis];
			}
		}
  }
  else if(count==15*200){
    
		for(axis=0;axis<AXIS_NUMBER;axis++)
		{
			//gyr_AVER[axis]=gyr_AVER[axis]/(5.f*200.f);
			if(GetCommand()&ACCUMULATE)
			{
				count++;
				gyr_AVER[axis]=allPara.GYRO_Real[axis];
			}
			for(gyro=0;gyro<GYRO_NUMBER;gyro++)
			{
				accInit[gyro][axis]=accInit[gyro][axis]/(5.f*200.f);
			}
		}
    allPara.ACC_InitSum=0;
		for(gyro=0;gyro<GYRO_NUMBER;gyro++)
			for(axis=0;axis<AXIS_NUMBER;axis++)
				allPara.ACC_InitSum=allPara.ACC_InitSum+accInit[gyro][axis]*accInit[gyro][axis];
		
    /* ��ȡ���ٶȵ�ֵ */
    icm_update_AccRad(accInit);
    Euler_to_Quaternion(allPara.ACC_RealAngle,allPara.quarternion);
  }
}


void updateAngle(void)
{	
  static float euler[3];            //ŷ����
	//float maxStaticValue=*(flashData.minValue);
	
//  if((GetCommand()&STATIC)&&(allPara.GYRO_Real[2]<0.2f)){
//		maxStaticValue=0.15f;
//	}
	
//  if(fabs(allPara.GYRO_Real[0])<maxStaticValue)//��λ ��/s
//    allPara.GYRO_Real[0]=0.f;	
//  if(fabs(allPara.GYRO_Real[1])<maxStaticValue)//��λ ��/s
//    allPara.GYRO_Real[1]=0.f;	
//  if(fabs(allPara.GYRO_Real[2])<maxStaticValue)//��λ ��/s
//    allPara.GYRO_Real[2]=0.f;
	
  /*���ٶȻ��ֳ���Ԫ��*/
  QuaternionInt(allPara.quarternion,allPara.GYRO_Real);
  /* ��Ԫ��ת����ŷ���� */
  Quaternion_to_Euler(allPara.quarternion,euler);

//  if(JudgeAcc()){
//    euler[0]=allPara.ACC_Angle[0];
//		euler[1]=allPara.ACC_Angle[1];
//    quaternion=Euler_to_Quaternion(euler);
//  }
	
  /*���ȽǶ�ת�� */
//  allPara.Result_Angle[0]= euler[0]/PI*180.0f;
//  allPara.Result_Angle[1]= euler[1]/PI*180.0f;
		allPara.Result_Angle[2]=-euler[2]/PI*180.0f;
}

void SetAngle(float angle){
	float euler[3];
	euler[0]=0.0f;
	euler[1]=0.0f;
	euler[2]=angle/180.f*PI;
	Euler_to_Quaternion(euler,allPara.quarternion);
}
float getActIcm(void){
	return (float)allPara.GYRO_Aver[2];
}
/**
* @brief  �õ�������ĽǶ�
* @param  none
* @retval ����ĽǶ�
*/
void getAngle(float angle[3])
{
	for(int i=0;i<3;i++)
		angle[i]=allPara.Result_Angle[i];
}

int JudgeAcc(void)
{
	float sum[GYRO_NUMBER]={0.f};
	float accInitSum=0.f;
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		float X_G,Y_G,Z_G;
		sum[gyro]=sqrt(allPara.ACC_Aver[gyro][0]*allPara.ACC_Aver[gyro][0]+allPara.ACC_Aver[gyro][1]*allPara.ACC_Aver[gyro][1]+allPara.ACC_Aver[gyro][2]*allPara.ACC_Aver[gyro][2]);
		
		X_G=allPara.ACC_Aver[gyro][0]/sum[gyro];
		Y_G=allPara.ACC_Aver[gyro][1]/sum[gyro];
		Z_G=allPara.ACC_Aver[gyro][2]/sum[gyro];
		/*��ʼ����Ϊ0,0,g,Ȼ�����ͨ������任��ʽ�����Ƶ�*/
		allPara.ACC_Angle[gyro][1]= safe_atan2( X_G , -Z_G);
		allPara.ACC_Angle[gyro][0]=-safe_atan2( Y_G , X_G/sin(allPara.ACC_Angle[gyro][1]));
	}
	allPara.ACC_RealAngle[0]=(allPara.ACC_Angle[0][0]+allPara.ACC_Angle[1][0]+allPara.ACC_Angle[2][0])/3.f;
	allPara.ACC_RealAngle[1]=(allPara.ACC_Angle[0][1]+allPara.ACC_Angle[1][1]+allPara.ACC_Angle[2][1])/3.f;
	
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		for(int axis=0;axis<AXIS_NUMBER;axis++)
			accInitSum=accInitSum+allPara.ACC_Aver[gyro][axis]*allPara.ACC_Aver[gyro][axis];
  if(fabs(accInitSum-allPara.ACC_InitSum)<0.003)
    return 1;
  else
    return 0;
}	

/*
chartMode��ʽ  
������1��X��  Y��  Z�ᣩ
������2��X��  Y��  Z�ᣩ
������3��X��  Y��  Z�ᣩ

chartMode+��������ţ�0-2��*3+��ţ�0-2��
*/

//#define GYRO_NUMBER    									
//#define AXIS_NUMBER    									
//#define TEMP_SAMPLE_NUMBER    					

/*
chartSelect��ʽ  
������1��X�ᣨTEMP_SAMPLE_NUMBER�������Y�ᣨTEMP_SAMPLE_NUMBER�������Z�ᣨTEMP_SAMPLE_NUMBER���������
������2��X�ᣨTEMP_SAMPLE_NUMBER�������Y�ᣨTEMP_SAMPLE_NUMBER�������Z�ᣨTEMP_SAMPLE_NUMBER���������
������3��X�ᣨTEMP_SAMPLE_NUMBER�������Y�ᣨTEMP_SAMPLE_NUMBER�������Z�ᣨTEMP_SAMPLE_NUMBER���������

chartSelect+��������ţ�0-(GYRO_NUMBER-1��*AXIS_NUMBER*+��ţ�0-(AXIS_NUMBER-1��*TEMP_SAMPLE_NUMBER+����ţ�0-(TEMP_SAMPLE_NUMBER-1)��
*/
const double stdCoffeicent[GYRO_NUMBER][AXIS_NUMBER]={ 0.0,0.0, 0.0 };

void driftCoffecientInit(void){
	int selectCount[GYRO_NUMBER][AXIS_NUMBER]={0};
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		for(int axis=0;axis<AXIS_NUMBER;axis++)
		{
			switch(*(flashData.chartMode+gyro*AXIS_NUMBER+axis))
			{
				//���֮ǰ��õı�׼����
				case 0:
					selectCount[gyro][axis]++;
					allPara.driftCoffecient[gyro][axis]=stdCoffeicent[gyro][axis];
					break;
				//�����֮ǰ��õı�׼����
				case 1:
					
					break;
			}
			for(int sample=0;sample<TEMP_SAMPLE_NUMBER;sample++)
			{
				if(*( flashData.chartSelect+gyro*AXIS_NUMBER*TEMP_SAMPLE_NUMBER+axis*TEMP_SAMPLE_NUMBER+sample) )
				{
					selectCount[gyro][axis]++;
					allPara.driftCoffecient[gyro][axis]=allPara.driftCoffecient[gyro][axis]+*(flashData.chartW+gyro*AXIS_NUMBER*TEMP_SAMPLE_NUMBER+axis*TEMP_SAMPLE_NUMBER+sample);
				}
			}
			allPara.driftCoffecient[gyro][axis]=allPara.driftCoffecient[gyro][axis]/selectCount[gyro][axis];	
		}
	}
	
#ifdef TEST_SUMMER
#endif
}

float safe_asin(float v)
{
	int error=0;
	if (isnan(v)) 
		error=1;
	else if (v >= 1.0f) 
		error=2;
	else if (v <= -1.0f) 
		error=3;
	
	if(error!=0)
	{
		uint32_t r_sp ;
			/*�жϷ����쳣ʱʹ��MSP����PSP*/		
		if(__get_PSP()!=0x00) //��ȡSP��ֵ
			r_sp = __get_PSP(); 
		else
			r_sp = __get_MSP(); 
		/*��Ϊ�����жϺ�����ջ֮�󣬶�ջָ����С0x10������ƽ�ƻ��������ܲ������ձ��ԣ�*/
		r_sp = r_sp+0x10;
		/*���ڷ���֪ͨ*/
		USART_OUT(USART1,"sinFault %d",error);
		char sPoint[2]={0};
		USART_OUT(USART1,"%s","0x");
		/*��ȡ�����쳣ʱ����ĵ�ַ*/
		for(int i=3;i>=-28;i--){
			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
			USART_OUT(USART1,"%s",sPoint);
			if(i%4==0)
				USART_Enter();
		}
		/*���ͻس���*/
		USART_Enter();
		switch(error)
		{
			case 1:
				return 0.0;
			case 2:
				return 3.1415926/2;
			case 3:
				return -3.1415926/2;
		}
	}else
			return asin(v);
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
	int error=0;

		if (isnan(y)) 
	{ 
		error=1;
  }else if(isnan(x/y))
	{
		if(x>0)
			error=2;
		else if(x<0)
			error=3;
		else 
			error=4;
	}
	
	if(error!=0)
	{
		uint32_t r_sp ;
			/*�жϷ����쳣ʱʹ��MSP����PSP*/		
		if(__get_PSP()!=0x00) //��ȡSP��ֵ
			r_sp = __get_PSP(); 
		else
			r_sp = __get_MSP(); 
		/*��Ϊ�����жϺ�����ջ֮�󣬶�ջָ����С0x10������ƽ�ƻ��������ܲ������ձ��ԣ�*/
		r_sp = r_sp+0x10;
		/*���ڷ���֪ͨ*/
		USART_OUT(USART1,"tanFault %d",error);
		char sPoint[2]={0};
		USART_OUT(USART1,"%s","0x");
		/*��ȡ�����쳣ʱ����ĵ�ַ*/
		for(int i=3;i>=-28;i--){
			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
			USART_OUT(USART1,"%s",sPoint);
			if(i%4==0)
				USART_Enter();
		}
		/*���ͻس���*/
		USART_Enter();
		switch(error)
		{
			case 1:
				return 0.0f;
			case 2:
				return  3.1415926/2.0; 
			case 3:
				return -3.1415926/2.0;
			case 4:
				return 0.0;
		}
	}else
		return atan2(x,y);
	return atan2(x,y);
}

/*�㷨�� H,��,����Ϊһ*/
double KalmanFilterZ1(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
  static double P_last=0.01;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.003;       //ϵͳ����        
  double R=(double)(flashData.varXYZ)[2];      //�������� 
  static double IAE_st[50];    //��¼����Ϣ
  static double data=0.0;
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
double KalmanFilterZ(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
  static double P_last=0.01;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.003;       //ϵͳ����        
  double R=(double)(flashData.varXYZ)[2];      //�������� 
  static double IAE_st[50];    //��¼����Ϣ
  static double data=0.0;
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
  double R=(double)(flashData.varXYZ)[0];      //�������� 
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
  
  static double P_last=0.01;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.003;       //ϵͳ����        
  double R=(double)(flashData.varXYZ)[1];      //�������� 
  static double IAE_st[50];    //��¼����Ϣ
  static double data=0.0;
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


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
