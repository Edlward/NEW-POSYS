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

#ifdef AUTOCAR
#define TIME_STATIC					(20)
#define TIME_STATIC_REAL		(TIME_STATIC-(2))
#else
#define TIME_STATIC					(5)
#define TIME_STATIC_REAL		(TIME_STATIC-(2))
#endif

/*������룬���С������������е�����*/
#define STATIC_MAX_NUM	1000
#define STATIC_MIN_NUM	600
double lowpass=0.0;
int RoughHandle(void)
{
  static int ignore=0;
	
  allPara.GYRO_Real[0]=(allPara.sDta.GYRO_Aver[0]);
  allPara.GYRO_Real[1]=(allPara.sDta.GYRO_Aver[1]);
  allPara.GYRO_Real[2]=(allPara.sDta.GYRO_Aver[2]);
	
	lowpass=LowPassFilterGyro(allPara.sDta.GYRO_Aver[2]);
	if(allPara.resetFlag)
		ignore=TIME_STATIC_REAL*200+1;
	
	ignore++;
	
  if(ignore>(TIME_STATIC_REAL)*200){
		//�����Ʈ���ݲɼ����
		if(UpdateBais())
		{
			allPara.GYRO_Real[0]=(double)(allPara.GYRO_Real[0]-allPara.sDta.GYRO_Bais[0]);
			allPara.GYRO_Real[1]=(double)(allPara.GYRO_Real[1]-allPara.sDta.GYRO_Bais[1]);
			allPara.GYRO_Real[2]=(double)(allPara.GYRO_Real[2]-allPara.sDta.GYRO_Bais[2]);
			allPara.kalmanZ=KalmanFilterZ(allPara.GYRO_Real[2]);
			return 1;
		}
		else
		{
			ignore=ignore;
			return 0;
		}
		
		
//		if(ignore>(TIME_STATIC_REAL)*200+STATIC_MAX_NUM+100)
//		{
////			if(abs(allPara.sDta.vell[0])>5||abs(allPara.sDta.vell[1])>5)
////				SetFlag(START_COMPETE);
//			ignore=10000;
//			return 1;
//		}
  }else{
		 return 0;
	}
}

#define PLAT_TIME		(10)
#define PLAT_NUM		(PLAT_TIME*200)
float PlatFilter(float newValue)
{
	/*�����˲�����õ�������*/
	static float value=0.f;
	/*����ƽ���˲�������*/
	static float data[PLAT_NUM]={0.f};
	/*��¼�����������*/
	static int index=0;
	
	if(index<PLAT_NUM)
	{
		data[index]=newValue;
		value=value+newValue*1.f/PLAT_NUM;
		index++;
	}
	//����������ֵ����ȥ��һ��ֵ������
	else if(index>=STATIC_MAX_NUM)
	{
		value=value-data[0]*1.f/PLAT_NUM;
		for(int i=0;i<STATIC_MAX_NUM-1;i++)
			data[i]=data[i+1];
		data[STATIC_MAX_NUM-1]=newValue;
		value=value+data[STATIC_MAX_NUM-1]*1.f/PLAT_NUM;
	}
	
	return value;
}

void updateAngle(void)
{	
//	float maxStaticValue=*(flashData.minValue);
//	
//  if((allPara.sDta.flag&STATIC)&&(allPara.GYRO_Real[2]<0.2f)){
//		maxStaticValue=0.15f;
//	}
	double w[3]={0.0};
	
	w[0]=allPara.GYRO_Real[0];
	w[1]=allPara.GYRO_Real[1];
	w[2]=allPara.GYRO_Real[2];
	
  if(fabs(w[0])<0.3f)//��λ ��/s
    w[0]=0.f;
  if(fabs(w[1])<0.3f)//��λ ��/s
    w[1]=0.f;
	
	#ifdef AUTOCAR
//  if((allPara.sDta.flag&STATIC_FORCE)||(abs(allPara.sDta.vell[0])<=1&&abs(allPara.sDta.vell[1])<=1&&fabs(allPara.kalmanZ)<0.05))//��λ ��/s
  if(allPara.sDta.flag&STATIC_FORCE)//��λ ��/s
    w[2]=0.f;
	#else
	if((allPara.sDta.flag&STATIC_FORCE)||(abs(allPara.sDta.vell[0])<=1&&abs(allPara.sDta.vell[1])<=1))
    w[2]=0.f;
	#endif
	allPara.sDta.Result_Angle[2]+=w[2]*0.005;
	
	if(allPara.sDta.Result_Angle[2]>180.0)
		allPara.sDta.Result_Angle[2]-=360.0;
	else if(allPara.sDta.Result_Angle[2]<-180.0)
		allPara.sDta.Result_Angle[2]+=360.0;
	
}



void SetAngle(float angle){
	float euler[3];
	euler[0]=0.0f;
	euler[1]=0.0f;
	euler[2]=angle/180.f*PI;
	Euler_to_Quaternion(euler,allPara.sDta.quarternion);
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

#define STATIC_ARRAY_NUM	20

uint16_t FindMax(uint16_t codes[STATIC_ARRAY_NUM])
{
	uint16_t Max=codes[0]; 
  for(int i=1;i<STATIC_ARRAY_NUM;i++)
	{
		if(codes[i]>Max) Max=codes[i];
	}
	return Max;
}

uint16_t FindMin(uint16_t codes[STATIC_ARRAY_NUM])
{
	uint16_t Min=codes[0]; 
  for(int i=1;i<STATIC_ARRAY_NUM;i++)
	{
		if(codes[i]<Min) Min=codes[i];
	}
	return Min;
}

void JudgeStatic(void)
{
	static uint16_t codes0[STATIC_ARRAY_NUM];
	static uint16_t codes1[STATIC_ARRAY_NUM];
	
	for(int i=0;i<STATIC_ARRAY_NUM-1;i++)
	{
		codes0[i]=codes0[i+1];
		codes1[i]=codes1[i+1];
	}
	codes0[STATIC_ARRAY_NUM-1]=allPara.sDta.codeData[0];
	codes1[STATIC_ARRAY_NUM-1]=allPara.sDta.codeData[1];

	
	//���������Ķ��ӣ��Ǿ�˵��ǿ�ƾ�ֹ�д�
	if(abs(FindMin(codes0)-FindMax(codes0))>=3||abs(FindMin(codes1)-FindMax(codes1))>=3||allPara.GYRO_Real[2]>0.5f)
	{
		SetFlag(~STATIC_FORCE);
	}
	
	#ifndef AUTOCAR
//	if(abs(FindMin(codes0)-FindMax(codes0))<=1&&abs(FindMin(codes1)-FindMax(codes1))<=1)
//		SetFlag(STATIC_FORCE);
	#endif
	
}

uint8_t UpdateBais(void)
{  
	static int index=0;
  static double data[AXIS_NUMBER][STATIC_MAX_NUM]={0.f};    	//������
	static int cnt=0;
	
	uint8_t returnValue=0;
	
	if(allPara.resetFlag)
		cnt=STATIC_MIN_NUM+1;
	
	if(cnt<=STATIC_MIN_NUM)
		cnt++;
	
	if(cnt>STATIC_MIN_NUM)
		returnValue=1;
	else
		returnValue=0;
	
	/*����һ	������û��ʼ*/
	if((!(allPara.sDta.flag&START_COMPETE))\
		/*������	�ռ�����������*/
		||(cnt<=STATIC_MIN_NUM)\
		/*������	ǿ�ƽ��뾲ֹ״̬�������������ã��������жϣ�*/
		||(allPara.sDta.flag&STATIC_FORCE))
	{
		index++;
		/*����Ѵ�����С�ڵ�����󳤶ȣ�˳��һ��һ���洢*/
		if(index<=STATIC_MAX_NUM)
		{
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				data[axis][index-1]=allPara.sDta.GYRO_Aver[axis];
//				if(axis==2)
//					data[axis][index-1]=lowpass;
			}
		}
		//����������ֵ����ȥ��һ��ֵ������
		else if(index>STATIC_MAX_NUM)
		{
			index=STATIC_MAX_NUM;
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				//����ǰ��һλ
				if(fabs(allPara.sDta.GYRO_Aver[axis])<4.f&&!isnan(allPara.sDta.GYRO_Aver[axis]))
				{
					for(int i=0;i<STATIC_MAX_NUM-1;i++)
						data[axis][i]=data[axis][i+1];
					data[axis][STATIC_MAX_NUM-1]=allPara.sDta.GYRO_Aver[axis];
//					if(axis==2)
//						data[axis][STATIC_MAX_NUM-1]=lowpass;
				}
			}
		}
	}
	//����ֹ�ˣ������������
	else 
	{
		double sum[AXIS_NUMBER] = {0.0};
		/*������ֵ*/
		if(index>=STATIC_MIN_NUM)
		{
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				for(int i=0;i<index-1;i++)
				{
					sum[axis]=sum[axis]+data[axis][i];
				}
				#ifdef AUTOCAR
				if((fabs(sum[axis]/(index-1)-allPara.sDta.GYRO_Bais[axis])<0.009&&allPara.sDta.GYRO_Bais[axis]!=0.0)||(allPara.sDta.GYRO_Bais[axis]==0.0))
					allPara.sDta.GYRO_Bais[axis]=sum[axis]/(index-1);
				#else
				if((fabs(sum[axis]/(index-1)-allPara.sDta.GYRO_Bais[axis])<0.02&&allPara.sDta.GYRO_Bais[axis]!=0.0)||(allPara.sDta.GYRO_Bais[axis]==0.0))
					allPara.sDta.GYRO_Bais[axis]=sum[axis]/(index-1);
				#endif
			}
		}
		for(int axis=0;axis<AXIS_NUMBER;axis++)
			for(int i=0;i<STATIC_MAX_NUM;i++)
				data[axis][i]=0.0;
		index=0;
	}
	
	return returnValue;
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
/*�㷨�� H,��,����Ϊһ*/
double KalmanFilterZ1(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
  static double P_last=0.00001;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.00001;       //ϵͳ����        
  double R=0.001;      //�������� 
  static double data=0.0;
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
  predict=act_value;
  
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return measureData;
  }
  else if(ignore==100){
    predict=data/99.0;
  }else
    ignore=101;
  
  /* Ԥ�Ȿ�ε�Ԥ����� */
  P_mid=P_last+Q;
  
  /* ����ϵ����������ֵ */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* ����Ԥ����� */
  P_last=(1-Kk)*P_mid;
  
  return act_value;
}

/*�㷨�� H,��,����Ϊһ*/
double KalmanFilterZ(double measureData)
{
  static double act_value=0;  //ʵ��ֵ
  double predict;             //Ԥ��ֵ
  
  static double P_last=0.00001;   //��һ�ε�Ԥ�����
  static double P_mid;        //��Ԥ������Ԥ��
  static double Kk;           //�˲�����ϵ��
  
  static double Q=0.00001;       //ϵͳ����        
  double R=0.001;      //�������� 
  static double data=0.0;
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
  predict=act_value;
  
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return measureData;
  }
  else if(ignore==100){
    predict=data/99.0;
  }else
    ignore=101;
  
  /* Ԥ�Ȿ�ε�Ԥ����� */
  P_mid=P_last+Q;
  
  /* ����ϵ����������ֵ */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* ����Ԥ����� */
  P_last=(1-Kk)*P_mid;
  
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
