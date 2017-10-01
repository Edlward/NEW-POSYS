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
/*  全局变量  */
float K_acc=1;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
double KalmanFilterZ(double measureData);
double KalmanFilterY(double measureData);
double KalmanFilterX(double measureData);


/**
  * @brief  更新角度值
  *
  * @param[in] gyrData,输入陀螺仪的加速度值
  *            magData,输入磁力计的值
  *            accData,输入加速度的值
  * @retval 初始化完成的标志位
  */
	
static three_axis_d gyr_act;			//原始的角速度和处理后的角速度
static gyro_t acc_icm;          //加速度信息
static float   temp_icm;            //陀螺仪温度
int chartIndex = 0;
static float gyr_AVER[3][TempTable_Num]={0.0};
void RoughHandle(void)
{
  gyro_t gyr_icm={0.f};
	static float proportion=0.f;
	/*跟新传感器的数据*/
	icm_update_gyro_rate();
	icm_update_temp();
	
	/*读取数据*/
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

	USART_OUT_F(temp_icm);
	USART_OUT_F(gyr_icm.No1.z);
	/* 新息自适应卡尔曼滤波，滤除角速度中的噪声 */
	gyr_act.x=KalmanFilterX(gyr_icm.No1.x);
	gyr_act.y=KalmanFilterY(gyr_icm.No1.y);
	gyr_act.z=KalmanFilterZ(gyr_icm.No1.z);

	USART_OUT_F(gyr_act.z);
	USART_Enter();
	/* 要放在滤波之后，因为不能让去温飘影响滤波参数（自适应温飘之后会有突降）*/
	/* 温度值转换成数组的序号来寻找温度零漂表里对应的值 */
	chartIndex=roundf((temp_icm-TempTable_min)*10);
	if((chartIndex>=0&&chartIndex<TempTable_Num-1)
	    &&chartNum[chartIndex]>=LEASTNUM           //表里的值得数量需要大于LEASTNUM
	    &&chartNum[chartIndex]!=0xffffffff) //表里面需要有值,原因flash默认都是1
	{
		  proportion=(temp_icm-TempTable_min)*10-chartIndex;
			/*分段插值*/
			gyr_act.x=(double)(gyr_icm.No1.x-(gyr_AVER[0][chartIndex]*(1-proportion)+gyr_AVER[0][chartIndex+1]*proportion));
			gyr_act.y=(double)(gyr_icm.No1.y-(gyr_AVER[1][chartIndex]*(1-proportion)+gyr_AVER[1][chartIndex+1]*proportion));
			gyr_act.z=(double)(gyr_icm.No1.z-(gyr_AVER[2][chartIndex]*(1-proportion)+gyr_AVER[2][chartIndex+1]*proportion));
	}
  else /* 当温度表里信息不理想时采用最小二乘法来拟合曲线参与零点漂移的计算  */
	{
		gyr_act.z=gyr_icm.No1.z-FitResult(temp_icm);
	}
}
float getTemp_icm(void){
	return temp_icm;
}
void aver(double gyr_aver[3][TempTable_Num],double countTemp[TempTable_Num]);
static three_axis acc_angle;        //加速度计测出的对应的角度
	//30℃到49.9℃   重新矫正温飘  30 -- 0  30.1 -- 1
void TemporaryHandle(void)
{
	static double gyr_aver[3][TempTable_Num]={0.0};
	static double countTemp[TempTable_Num]={0.0};
	static double accInit[3]={0.0,0.0,0.0};
	static uint32_t count=0;
	static uint32_t flag=0;
	
	//忽略前五十个数
	static uint32_t ignore=0;
	ignore++;
	if(ignore<50)
		return ;
	else
		ignore=51;
	
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
			aver(gyr_aver,countTemp);
			for(int i=0;i<TempTable_Num;i++){
						USART_OUT_F(gyr_aver[2][i]);
						USART_Enter();
					for(uint32_t j=0;j<3;j++){
						gyr_AVER[j][i]=gyr_AVER[j][i]+gyr_aver[j][i];
						gyr_aver[j][i]=0.0;
					}
					countTemp[i]=0.0;
			}
			SquareFitting(gyr_AVER[2]);
			for(uint32_t i=0;i<3;i++)
				accInit[i]=accInit[i]/count;
			
			/* 读取加速度的值 */
			icm_update_AccRad(accInit,&acc_angle);
			quaternion=Euler_to_Quaternion(acc_angle);
		}
}
void aver(double gyr_aver[3][TempTable_Num],double countTemp[TempTable_Num]){
	double sum[3]={0.0};
	uint32_t count=0;
	for(int i=0;i<TempTable_Num;i++){
		sum[0]=sum[0]+gyr_aver[0][i];
		sum[1]=sum[1]+gyr_aver[1][i];
		sum[2]=sum[2]+gyr_aver[2][i];
		count=count+countTemp[i];
	}
	sum[0]=sum[0]/count;
	sum[1]=sum[1]/count;
	sum[2]=sum[2]/count;
	for(int i=0;i<TempTable_Num;i++){
		gyr_aver[0][i]=sum[0];
		gyr_aver[1][i]=sum[1];
		gyr_aver[2][i]=sum[2];
	}
}
	

uint8_t updateAngle(void)
{	
	static three_axis euler;            //欧垃角
	static three_axis result;									//最终角度的弧度角
	
	/* 
	阈值 低于此值则认为没动
	*/
//	if(fabs(gyr_act.z)<0.05)//单位 °/s
//		gyr_act.z=0;
	
	/*角速度积分成四元数*/
	/*
	选用ZXY旋转顺序,因为这样符合人们对航向角和俯仰角的认识
	所谓以何种顺序旋转,可以有两种理解.
	一是从顺序旋转欧拉角再生成四元数,可谓旋转顺序不同,最后姿态不同,四元数也不同
	二是从以固定角速度把四元数积分成一个固定姿态,再推倒到欧拉角,以何种顺序完全取决人们自己的认识
	x正负90,yz正负180.
	*/
	quaternion=QuaternionInt(quaternion,gyr_act);
	/*
	三个轴是相互耦合的,能够互相影响
	*/
  /* 四元数转换成欧垃角 */	
	euler=Quaternion_to_Euler(quaternion);
	
//	/* 融合加速度计和陀螺仪 */
//	result.x=acc_angle.x;
//	result.y=acc_angle.y;
//	result.z=euler.z;
//	/* 回到四元数 */
//	quaternion=Euler_to_Quaternion(result);
	
//	/*弧度角度转换 */
//	result_angle.x= result.x/PI*180.0f;
//	result_angle.y= result.y/PI*180.0f;
	result_angle.z=-euler.z/PI*180.0f;
	
	return 1;
}


/**
  * @brief  得到计算出的角度
  * @param  none
  * @retval 计算的角度
  */
three_axis getAngle(void)
{
	return result_angle;
}
/**
  * @brief  优化后的反三角函数
  * @param  v: 对应的sin值
  * @retval 得到v反正弦的值
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
  * @brief  优化后的反三角函数
  * @param  x: tan=x/y
  * @param  y:
  * @retval 得到反正切的值
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
/*算法中 H,φ,Γ均为一*/
double KalmanFilterZ(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.00000001;       //系统噪声         
	static double R=0.006708655;      //测量噪声 
	static double IAE_st[50];    //记录的新息
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
	
	
	/* 预测本次的预测误差 */
	P_mid=P_last+Q;
	
	/* 计算系数，求得输出值 */
	Kk=P_mid/(P_mid+R);
	
	act_value=predict+Kk*(measureData-predict);
	
	/* 更新预测误差 */
	P_last=(1-Kk)*P_mid;
	
	/* 计算并调整系统噪声 */
	//Q=Kk*Kk*Cr;

	/* 为提高滤波器的响应速度，减小滞后而设下的阈值 */
	if(Kk>0.5)
		act_value=measureData;
	
	return act_value;
}
/*算法中 H,φ,Γ均为一*/
double KalmanFilterX(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.011205;       //系统噪声         
	static double R=0.011205;      //测量噪声 
	static double IAE_st[50];    //记录的新息
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
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.01010547;       //系统噪声         
	static double R=0.01010547;      //测量噪声 
	static double IAE_st[50];    //记录的新息
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


float KalmanFilterT(double measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.025;       //系统噪声
	static double R=0.025;      //测量噪声         
	
	static double IAE_st[50];    //记录的新息
	double Cr=0;                //新息的方差
	
	/* 获得来自用户的校正值，如果用户不矫正，则使用0.012 */
	//R=Get_R_Zaxis();
   
	predict=act_value;
	
	//相当于数组的左移
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;
	
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
	
	return (float)act_value;
}




/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
