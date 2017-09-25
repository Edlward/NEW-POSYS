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
extern float *chartW;
extern uint32_t *chartNum;
extern int chartN;
static three_axis result_angle={0,0,0};
static Quarternion quaternion={1,0,0,0};
/* Extern   variables ---------------------------------------------------------*/
/*  全局变量  */
float K_acc=1;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
float KalmanFilterT(float measureData);
float KalmanFilterZ(float measureData);
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/



/**
  * @brief  更新角度值
  *
  * @param[in] gyrData,输入陀螺仪的加速度值
  *            magData,输入磁力计的值
  *            accData,输入加速度的值
  * @retval 初始化完成的标志位
  */
	
static gyro_t gyr_icm,gyr_act;			//原始的角速度和处理后的角速度
static gyro_t acc_icm;          //加速度信息
static float   temp_icm;            //陀螺仪温度
static float   temp_icm_filter;            //陀螺仪温度
int chartIndex = 0;
void RoughHandle(void)
{
	/*跟新传感器的数据*/
	icm_update_gyro_rate();
	icm_update_temp();
	//icm_update_acc();
	
	/*读取数据*/
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	//icm_read_accel_acc(&acc_icm);
	
	temp_icm=KalmanFilterT(temp_icm);

	/* 温度值转换成数组的序号来寻找温度零漂表里对应的值 */
	chartIndex=roundf((temp_icm-30.f)*10);
	if((chartIndex>=0&&chartIndex<chartN)
	    &&chartNum[chartIndex]>=LEASTNUM           //表里的值得数量需要大于LEASTNUM
	    &&chartNum[chartIndex]!=0xffffffff) //表里面需要有值,原因flash默认都是1
	{
		gyr_act.No1.z=gyr_icm.No1.z-chartW[chartIndex];
	}
  else /* 当温度表里信息不理想时采用最小二乘法来拟合曲线参与零点漂移的计算  */
	{
		gyr_act.No1.z=FitResult(temp_icm);
	}
}
	//30℃到49.9℃   重新矫正温飘  30 -- 0  30.1 -- 1
void TemporaryHandle(int start)
{
	static float gyr_aver[((TempTable_max - TempTable_min)*10)]={0};
	static float countTemp[((TempTable_max - TempTable_min)*10)]={0};
	chartIndex=roundf((temp_icm-30.f)*10);
	if(chartIndex<0||chartIndex>=chartN) 
		return;
	if(start==0){
    if(countTemp[chartIndex]>0)
			gyr_aver[chartIndex]=gyr_aver[chartIndex]*countTemp[chartIndex]/(countTemp[chartIndex]+1)+gyr_act.No1.z/(countTemp[chartIndex]+1);
		else
			gyr_aver[chartIndex]=gyr_act.No1.z;
		
			countTemp[chartIndex]++;
	}else{
		if(countTemp[chartIndex]>=LEASTNUM){
			float proportion=(temp_icm-30.f)*10-chartIndex;
			/*分段插值*/
			gyr_act.No1.z=gyr_act.No1.z-(gyr_aver[chartIndex]*(1-proportion)+gyr_aver[chartIndex+1]*proportion);
		}
	}
}
	
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void UpdataExcel(void)
{
	uint8_t finish=0;
  while(!finish){
		while(getTimeFlag()){
			
		
		}
	}
}

static three_axis acc_angle;        //加速度计测出的对应的角度
uint8_t updateAngle(void)
{	
	static three_axis euler;            //欧垃角
	static uint8_t wait_flag=0;
	static uint8_t wait_second=0;
	static three_axis result;									//最终角度的弧度角
	
	/* 二次矫正处理 */
	/*
	为了减去零飘带来的影响
	初始化时取一秒的数据进行求平均值
	然后运动过程中时刻减去
	*/
	
	/* 新息自适应卡尔曼滤波，滤除角速度中的噪声 */
	gyr_act.No1.z=KalmanFilterZ(gyr_act.No1.z);
	/* 在车动之前不进行积分的标志位 */
	if(!wait_flag&&fabs(gyr_act.No1.z)>0.3)
	{
		wait_flag=1;
	}
		
	/* 
	阈值 低于此值则认为没动
	*/
	if(fabs(gyr_act.No1.z)<0.05)//单位 °/s
		gyr_act.No1.z=0;
	if(fabs(gyr_act.No1.x)<10)	
		gyr_act.No1.x=0;
	if(fabs(gyr_act.No1.y)<10)
		gyr_act.No1.y=0;
	
	/* 角度弧度转换 */
	gyr_act.No1.x=(gyr_act.No1.x)/180.0f*PI;
	gyr_act.No1.y=(gyr_act.No1.y)/180.0f*PI;
	gyr_act.No1.z=(gyr_act.No1.z)/180.0f*PI;
	
	/*角速度积分成四元数*/
	/*
	选用ZXY旋转顺序,因为这样符合人们对航向角和俯仰角的认识
	所谓以何种顺序旋转,可以有两种理解.
	一是从顺序旋转欧拉角再生成四元数,可谓旋转顺序不同,最后姿态不同,四元数也不同
	二是从以固定角速度把四元数积分成一个固定姿态,再推倒到欧拉角,以何种顺序完全取决人们自己的认识
	x正负90,yz正负180.
	*/
	 quaternion=QuaternionInt(quaternion,gyr_act.No1);
	/*
	三个轴是相互耦合的,能够互相影响
	*/
  /* 四元数转换成欧垃角 */	
	euler=Quaternion_to_Euler(quaternion);
	
	/* 读取加速度的值 */
	icm_update_AccRad(&acc_angle);
	
	/* 融合加速度计和陀螺仪 */
	result.x=euler.x*K_acc+acc_angle.x*(1-K_acc);
	result.y=euler.y*K_acc+acc_angle.y*(1-K_acc);
	result.z=euler.z;
	/* 回到四元数 */
	quaternion=Euler_to_Quaternion(result);
	
	/*弧度角度转换 */
	result_angle.x= result.x/PI*180.0f;
	result_angle.y= result.y/PI*180.0f;
	result_angle.z=-result.z/PI*180.0f;

	return wait_second;
}


//void Test(int finish){
//	
//	static float gyr_aver_rough[2000]={0};
//	static float gyr_aver[2000]={0};
//	static float countTemp[2000]={0};
//	static float countTemp_rough[2000]={0};
//	static uint32_t index=0;
//	static uint32_t index_rough=0;
//	static uint32_t count=0;
//	static int ii=0;
//	
//	if(finish==1&&ii<2000){
////			USART_OUT_F(gyr_aver_rough[ii]);
////			USART_OUT_F(countTemp_rough[ii]);
////			USART_OUT_F(gyr_aver[ii]);
////			USART_OUT_F(countTemp[ii]);
////			USART_OUT(USART6,"\r\n");
////			ii++;
//			return ;
//	}
//	
//	icm_update_gyro_rate();
//	icm_update_temp();
//	
//	icm_read_gyro_rate(&gyr_icm);
//	icm_read_temp(&temp_icm);
//	
//	USART_OUT_F(gyr_icm.No1.z);
//	USART_OUT_F(temp_icm);
//	gyr_act.No1.z=KalmanFilterZ(gyr_icm.No1.z);
//	temp_icm_filter=KalmanFilterT(temp_icm);
//	
//	gyr_act.No1.z=KalmanFilterZ(gyr_icm.No1.z);
//	temp_icm_filter=KalmanFilterT(temp_icm);
//	
//	if(temp_icm_filter>30.f){
//		index=(uint32_t)((temp_icm_filter-30.f)*100.0f/1.0f);
//		if(countTemp[index]>0){
//			gyr_aver[index]=gyr_aver[index]*countTemp[index]/(countTemp[index]+1)+gyr_act.No1.z/(countTemp[index]+1);
//		}
//		else{
//			gyr_aver[index]=gyr_act.No1.z;
//		}
//			
//			countTemp[index]++;
//	}
//	
//	if(temp_icm>30.f){
//		index_rough=(uint32_t)((temp_icm-30.f)*100.0f/1.0f);
//		if(countTemp_rough[index_rough]>0){
//			gyr_aver_rough[index_rough]=gyr_aver_rough[index_rough]*countTemp_rough[index_rough]/(countTemp_rough[index_rough]+1)+gyr_icm.No1.z/(countTemp_rough[index_rough]+1);
//		}
//		else{
//			gyr_aver_rough[index_rough]=gyr_icm.No1.z;
//		}
//			countTemp_rough[index_rough]++;
//	}
//	count++;
//	if(count==200){
//		count=0;
//		USART_OUT_F(temp_icm);
//		USART_OUT(USART6,"\r\n");
//	}
//}



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
float KalmanFilterZ(float measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.007;       //系统噪声         
	static double R=0.007f;      //测量噪声 
	static float IAE_st[50];    //记录的新息
	double Cr=0;                //新息的方差
	
  //令预测值为上一次的真实值
	predict=act_value;
	
	/* 新息的方差计算 */
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

float KalmanFilterT(float measureData)
{
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.025;       //系统噪声
	static double R=0.025;      //测量噪声         
	
	static float IAE_st[50];    //记录的新息
	double Cr=0;                //新息的方差
	
	/* 获得来自用户的校正值，如果用户不矫正，则使用0.012 */
	//R=Get_R_Zaxis();
   
	predict=act_value;
	
	//相当于数组的左移
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=measureData-predict;
	
	static uint8_t ignore=0;

	if(ignore++<50)
		return measureData;
	else
		ignore=50;
	
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
	
	return act_value;
}


//#define ACC_DEBUG
#define ICM_DEBUG
void DebugMode(void)
{
	#ifdef ACC_DEBUG
			/*弧度角度转换 */
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
