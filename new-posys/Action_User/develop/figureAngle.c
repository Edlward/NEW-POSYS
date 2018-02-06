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
int CalculateRealCrAndMean(float stdCr[AXIS_NUMBER],float mean[AXIS_NUMBER]);
int JudgeAcc(void);
/**
* @brief  ���½Ƕ�ֵ
*
* @param[in] gyrData,���������ǵļ��ٶ�ֵ
*            magData,��������Ƶ�ֵ
*            accData,������ٶȵ�ֵ
* @retval ��ʼ����ɵı�־λ
*/

int RoughHandle(void)
{
  static int ignore=0;
  static double data[AXIS_NUMBER][7*200]={0.f};    	//������
  static float stdCr[AXIS_NUMBER]={0.f};                //��Ϣ�ı�׼��
  static float mean[AXIS_NUMBER]={0.f};
	
  allPara.GYRO_Real[0]=(allPara.GYRO_Aver[0]);
  allPara.GYRO_Real[1]=(allPara.GYRO_Aver[1]);
  allPara.GYRO_Real[2]=(allPara.GYRO_Aver[2]);
	
	if(allPara.resetFlag)
		ignore=15*200+1;
	
  if((GetCommand()&ACCUMULATE)&&ignore>15*200){
    allPara.GYRO_Real[0]=(double)(allPara.GYRO_Real[0]-allPara.GYRO_Bais[0]);
    allPara.GYRO_Real[1]=(double)(allPara.GYRO_Real[1]-allPara.GYRO_Bais[1]);
    allPara.GYRO_Real[2]=(double)(allPara.GYRO_Real[2]-allPara.GYRO_Bais[2]);
		
    return 1;
  }else{
		 /*���ҷ���*/
		if(!CalculateRealCrAndMean(stdCr,mean))
			return 0;
		for(char axis=0;axis<AXIS_NUMBER;axis++)
    {
      if(fabs(allPara.GYRO_Real[axis]-mean[axis])>stdCr[axis]*3)
			return 0;
    }
		ignore++;
		for(int axis=0;axis<AXIS_NUMBER;axis++)
		{
			allPara.GYRO_Bais[axis]-=data[axis][0]/7.0/200.0;
			memcpy(data[axis],data[axis]+1,(7*200-1)*8);
			data[axis][7*200-1]=allPara.GYRO_Real[axis];
			allPara.GYRO_Bais[axis]+=data[axis][7*200-1]/7.0/200.0;
		}
		 return 0;
	}
}



void updateAngle(void)
{	
//	float maxStaticValue=*(flashData.minValue);
//	
//  if((GetCommand()&STATIC)&&(allPara.GYRO_Real[2]<0.2f)){
//		maxStaticValue=0.15f;
//	}
	
  if(fabs(allPara.GYRO_Real[0])<0.3f)//��λ ��/s
    allPara.GYRO_Real[0]=0.f;
  if(fabs(allPara.GYRO_Real[1])<0.3f)//��λ ��/s
    allPara.GYRO_Real[1]=0.f;
  if(fabs(allPara.GYRO_Real[2])<0.3f)//��λ ��/s
    allPara.GYRO_Real[2]=0.f;
	
	allPara.Result_Angle[2]+=allPara.GYRO_Real[2]*0.005;
	
	if(allPara.Result_Angle[2]>180.0)
		allPara.Result_Angle[2]-=360.0;
	else if(allPara.Result_Angle[2]<-180.0)
		allPara.Result_Angle[2]+=360.0;
	
}

int CalculateRealCrAndMean(float stdCr[AXIS_NUMBER],float mean[AXIS_NUMBER]){
  static float IAE_st[AXIS_NUMBER][200]={0.f};    //��¼����Ϣ
  static float data[AXIS_NUMBER][200]={0.f};    	//������
  static int ignore=0;
  ignore++;
  
  int axis=0;
  
  /* ��Ϣ�ķ������ */
    for(axis=0;axis<AXIS_NUMBER;axis++)
    {
      mean[axis]=mean[axis]-data[axis][0]/200;
      memcpy(data[axis],data[axis]+1,796);
      data[axis][199]=allPara.GYRO_Real[axis];
      memcpy(IAE_st[axis],IAE_st[axis]+1,796);
      mean[axis]=mean[axis]+data[axis][199]/200;
      IAE_st[axis][199]=allPara.GYRO_Real[axis]-mean[axis];
    }
  
  if(ignore<400)
    return 0;
  else
    ignore=400;
  
    for(axis=0;axis<AXIS_NUMBER;axis++){
      stdCr[axis]=0;
      for(int i=0;i<200;i++)
      {
        stdCr[axis]=stdCr[axis]+IAE_st[axis][i]*IAE_st[axis][i];
      }
      stdCr[axis]=__sqrtf(stdCr[axis]/200.0f);
    }
  
  return 1;
}


void SetAngle(float angle){
	float euler[3];
	euler[0]=0.0f;
	euler[1]=0.0f;
	euler[2]=angle/180.f*PI;
	Euler_to_Quaternion(euler,allPara.quarternion);
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
				if(*( flashData.chartSelect+gyro*AXIS_NUMBER*TEMP_SAMPLE_NUMBER+axis*TEMP_SAMPLE_NUMBER+sample))
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
