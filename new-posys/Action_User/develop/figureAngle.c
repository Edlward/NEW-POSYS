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
/*  全局变量  */
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
* @brief  更新角度值
*
* @param[in] gyrData,输入陀螺仪的加速度值
*            magData,输入磁力计的值
*            accData,输入加速度的值
* @retval 初始化完成的标志位
*/

#ifdef AUTOCAR
#define TIME_STATIC					(9)
#define TIME_STATIC_REAL		(TIME_STATIC-(2))
#else
#define TIME_STATIC					(9)
#define TIME_STATIC_REAL		(TIME_STATIC-(2))
#endif

/*最大两秒，如果小于两秒就用现有的数据*/
#define STATIC_MAX_NUM	200
#define STATIC_MIN_NUM	20
double kalmanZ=0.0;
int RoughHandle(void)
{
  static int ignore=0;
	
  allPara.GYRO_Real[0]=(allPara.GYRO_Aver[0]);
  allPara.GYRO_Real[1]=(allPara.GYRO_Aver[1]);
  allPara.GYRO_Real[2]=(allPara.GYRO_Aver[2]);
	
	if(allPara.resetFlag)
		ignore=TIME_STATIC_REAL*200+1;
	
	ignore++;
  if((GetCommand()&ACCUMULATE)&&ignore>(TIME_STATIC_REAL)*200){
    allPara.GYRO_Real[0]=(double)(allPara.GYRO_Real[0]-allPara.GYRO_Bais[0]);
    allPara.GYRO_Real[1]=(double)(allPara.GYRO_Real[1]-allPara.GYRO_Bais[1]);
    allPara.GYRO_Real[2]=(double)(allPara.GYRO_Real[2]-allPara.GYRO_Bais[2]);
		kalmanZ=KalmanFilterZ(allPara.GYRO_Real[2]);
		UpdateBais();
		if(ignore>(TIME_STATIC_REAL)*200+STATIC_MAX_NUM+100)
		{
			ignore=10000;
			return 1;
		}
		return 0;
  }else{
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
	double w[3]={0.0};
	
	w[0]=allPara.GYRO_Real[0];
	w[1]=allPara.GYRO_Real[1];
	w[2]=allPara.GYRO_Real[2];
	
  if(fabs(w[0])<0.3f)//单位 °/s
    w[0]=0.f;
  if(fabs(w[1])<0.3f)//单位 °/s
    w[1]=0.f;
	#ifdef AUTOCAR
  if(fabs(kalmanZ)<0.30f)//单位 °/s
    w[2]=0.f;
	#else
  if(fabs(kalmanZ)<0.15f)//单位 °/s
    w[2]=0.f;
	#endif
	allPara.Result_Angle[2]+=w[2]*0.005;
	
	if(allPara.Result_Angle[2]>180.0)
		allPara.Result_Angle[2]-=360.0;
	else if(allPara.Result_Angle[2]<-180.0)
		allPara.Result_Angle[2]+=360.0;
	
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
		/*初始坐标为0,0,g,然后可以通过坐标变换公式轻易推导*/
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

#define STATIC_ARRAY_NUM	5

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

static uint8_t key = 1;
void JudgeStatic(void)
{
	static uint16_t codes0[STATIC_ARRAY_NUM];
	static uint16_t codes1[STATIC_ARRAY_NUM];
	
	for(int i=0;i<STATIC_ARRAY_NUM-1;i++)
	{
		codes0[i]=codes0[i+1];
		codes1[i]=codes1[i+1];
	}
	codes0[STATIC_ARRAY_NUM-1]=allPara.codeData[0];
	codes1[STATIC_ARRAY_NUM-1]=allPara.codeData[1];

	if((abs(FindMin(codes0)-FindMax(codes0))<=1&&abs(FindMin(codes1)-FindMax(codes1))<=1)&&(fabs(allPara.GYRO_Real[2])<0.11f||key))
		allPara.isStatic=1;
	else
		allPara.isStatic=0;
}

void UpdateBais(void)
{  
	static int index=0;
  static double data[AXIS_NUMBER][STATIC_MAX_NUM]={0.f};    	//数据列
	
	/*如果判断成功静止，进行积累数据*/
	if(allPara.isStatic&&!(key==1&&index==STATIC_MAX_NUM))
	{
		index++;
		/*如果已存数据小于等于最大长度，顺序一个一个存储*/
		if(index<=STATIC_MAX_NUM)
		{
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				data[axis][index-1]=allPara.GYRO_Aver[axis];
			}
		}
		//如果超出最大值，减去第一个值，加入
		else if(index>STATIC_MAX_NUM)
		{
			index=STATIC_MAX_NUM;
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				//数组前移一位
				if(fabs(allPara.GYRO_Aver[axis])<4.f&&!isnan(allPara.GYRO_Aver[axis]))
				{
					for(int i=0;i<STATIC_MAX_NUM-1;i++)
						data[axis][i]=data[axis][i+1];
					data[axis][STATIC_MAX_NUM-1]=allPara.GYRO_Aver[axis];
				}
			}
		}
	}
	//不静止了，填入矫正参数
	else 
	{
		double sum[AXIS_NUMBER] = {0.0};
		/*计算总值*/
		if(index>=STATIC_MIN_NUM)
		{
			key=0;
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				for(int i=0;i<index-1;i++)
				{
					sum[axis]=sum[axis]+data[axis][i];
				}
				allPara.GYRO_Bais[axis]=sum[axis]/(index-1);
			}
		}
		for(int axis=0;axis<AXIS_NUMBER;axis++)
			for(int i=0;i<STATIC_MAX_NUM;i++)
				data[axis][i]=0.0;
		index=0;
	}
}



/*
chartMode方式  
陀螺仪1（X轴  Y轴  Z轴）
陀螺仪2（X轴  Y轴  Z轴）
陀螺仪3（X轴  Y轴  Z轴）

chartMode+陀螺仪序号（0-2）*3+轴号（0-2）
*/

//#define GYRO_NUMBER    									
//#define AXIS_NUMBER    									
//#define TEMP_SAMPLE_NUMBER    					

/*
chartSelect方式  
陀螺仪1（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））
陀螺仪2（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））
陀螺仪3（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））

chartSelect+陀螺仪序号（0-(GYRO_NUMBER-1）*AXIS_NUMBER*+轴号（0-(AXIS_NUMBER-1）*TEMP_SAMPLE_NUMBER+结果号（0-(TEMP_SAMPLE_NUMBER-1)）
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
				//结合之前测得的标准数据
				case 0:
					selectCount[gyro][axis]++;
					allPara.driftCoffecient[gyro][axis]=stdCoffeicent[gyro][axis];
					break;
				//不结合之前测得的标准数据
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
			/*判断发生异常时使用MSP还是PSP*/		
		if(__get_PSP()!=0x00) //获取SP的值
			r_sp = __get_PSP(); 
		else
			r_sp = __get_MSP(); 
		/*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
		r_sp = r_sp+0x10;
		/*串口发数通知*/
		USART_OUT(USART1,"sinFault %d",error);
		char sPoint[2]={0};
		USART_OUT(USART1,"%s","0x");
		/*获取出现异常时程序的地址*/
		for(int i=3;i>=-28;i--){
			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
			USART_OUT(USART1,"%s",sPoint);
			if(i%4==0)
				USART_Enter();
		}
		/*发送回车符*/
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
  * @brief  优化后的反三角函数
  * @param  x: tan=x/y
  * @param  y:
  * @retval 得到反正切的值
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
			/*判断发生异常时使用MSP还是PSP*/		
		if(__get_PSP()!=0x00) //获取SP的值
			r_sp = __get_PSP(); 
		else
			r_sp = __get_MSP(); 
		/*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
		r_sp = r_sp+0x10;
		/*串口发数通知*/
		USART_OUT(USART1,"tanFault %d",error);
		char sPoint[2]={0};
		USART_OUT(USART1,"%s","0x");
		/*获取出现异常时程序的地址*/
		for(int i=3;i>=-28;i--){
			Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
			USART_OUT(USART1,"%s",sPoint);
			if(i%4==0)
				USART_Enter();
		}
		/*发送回车符*/
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

/*算法中 H,φ,Γ均为一*/
double KalmanFilterZ1(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.01;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.003;       //系统噪声        
  double R=(double)(flashData.varXYZ)[2];      //测量噪声 
  static double IAE_st[50];    //记录的新息
  static double data=0.0;
  double Cr=0;                //新息的方差
  //令预测值为上一次的真实值
  predict=act_value;
  
  /* 新息的方差计算 */
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

/*算法中 H,φ,Γ均为一*/
double KalmanFilterZ(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.01;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.0001;       //系统噪声        
  double R=0.001;      //测量噪声 
  static double data=0.0;
  //令预测值为上一次的真实值
  predict=act_value;
  
  static uint32_t ignore=0;
  ignore++;
  if(ignore<100){
    data+=measureData;
    return 0.0;
  }
  else if(ignore==100){
    predict=data/99.0;
  }else
    ignore=101;
  
  /* 预测本次的预测误差 */
  P_mid=P_last+Q;
  
  /* 计算系数，求得输出值 */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* 更新预测误差 */
  P_last=(1-Kk)*P_mid;
  
  return act_value;
}
/*算法中 H,φ,Γ均为一*/
double KalmanFilterX(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.0000003169;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.00000000002514;       //系统噪声         
  double R=(double)(flashData.varXYZ)[0];      //测量噪声 
  static double IAE_st[50];    //记录的新息
  static double data=0.0;
  double Cr=0;                //新息的方差
  
  predict=act_value;
  
  /* 新息的方差计算 */
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

/*算法中 H,φ,Γ均为一*/
double KalmanFilterY(double measureData)
{
   static double act_value=0;  //实际值
  double predict;             //预测值
  
  static double P_last=0.01;   //上一次的预测误差
  static double P_mid;        //对预测误差的预测
  static double Kk;           //滤波增益系数
  
  static double Q=0.003;       //系统噪声        
  double R=(double)(flashData.varXYZ)[1];      //测量噪声 
  static double IAE_st[50];    //记录的新息
  static double data=0.0;
  double Cr=0;                //新息的方差
  //令预测值为上一次的真实值
  predict=act_value;
  
  /* 新息的方差计算 */
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
