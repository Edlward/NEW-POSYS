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
/*  全局变量  */
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
  * @brief  更新角度值
  *
  * @param[in] gyrData,输入陀螺仪的加速度值
  *            magData,输入磁力计的值
  *            accData,输入加速度的值
  * @retval 初始化完成的标志位
  */
static three_axis gyr_icm,gyr_act;  //原始的角速度和处理后的角速度
static three_axis acc_icm;          //加速度信息
static float   temp_icm;            //陀螺仪温度

void RoughHandle(void)
{
	static int16_t convert;             //陀螺仪温度对应数组序号  
	/*跟新传感器的数据*/
	icm_update_gyro_rate();
	icm_update_temp();
	icm_update_acc();
	//LIS3MDL_UpdateMag();
	
	/*读取数据*/
	icm_read_gyro_rate(&gyr_icm);
	icm_read_temp(&temp_icm);
	icm_read_accel_acc(&acc_icm);
	
	temp_icm=KalmanFilterT(temp_icm);
	
	a_icm=GetVdoff_icmErrArr();        //最小二乘拟合的结果

	/* 温度值转换成数组的序号来寻找温度零漂表里对应的值 */
	convert=(int16_t)((temp_icm-TempTable_min)*10.0f/1.0f);
	if((convert>=0&&convert<10*(TempTable_max-TempTable_min))
	    &&oldCountNum[convert]>10           //表里的值得数量需要大于10
	    &&oldCountNum[convert]!=0xffffffff) //表里面需要有值,原因flash默认都是1
	{
		gyr_act.x=(gyr_icm.x-oldResult[convert]);
		gyr_act.y=(gyr_icm.y-oldResult[convert+10*(TempTable_max-TempTable_min)]);
		gyr_act.z=(gyr_icm.z-oldResult[convert+20*(TempTable_max-TempTable_min)]);
	}
  else /* 当温度表里信息不理想时采用最小二乘法来拟合曲线参与零点漂移的计算  */
	{
		gyr_act.x=gyr_icm.x;
		gyr_act.y=gyr_icm.y;
		gyr_act.z=gyr_icm.z;
		/*减去一个一次函数*/
		for(uint8_t i=0;i<BF_TH;i++)
		{
			gyr_act.x=gyr_act.x-a_icm[i][0]*pow(temp_icm,i);
			gyr_act.y=gyr_act.y-a_icm[i][1]*pow(temp_icm,i);
			gyr_act.z=gyr_act.z-a_icm[i][2]*pow(temp_icm,i);
		}
	}
}

	//30℃到49.9℃   重新矫正温飘  30 -- 0  30.1 -- 1
void TemporaryHandle(int start)
{
	uint16_t index=(uint16_t)((temp_icm-30.f)*10.0f/1.0f);
	static float gyr_aver[200]={0};
	static float countTemp[200]={0};
	/* 新息自适应卡尔曼滤波，滤除角速度中的噪声 */
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
			/*分段插值*/
			gyr_act.z=gyr_act.z-(gyr_aver[index]*(1-proportion)+gyr_aver[index+1]*proportion);
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
	
	/* 在车动之前不进行积分的标志位 */
	if(!wait_flag&&fabs(gyr_act.z)>0.3)
	{
		wait_flag=1;
	}
		
	/* 
	阈值 低于此值则认为没动
	*/
	if(fabs(gyr_act.z)<0.05)//单位 °/s
		gyr_act.z=0;
	if(fabs(gyr_act.x)<10)	
		gyr_act.x=0;
	if(fabs(gyr_act.y)<10)
		gyr_act.y=0;
	
	/* 角度弧度转换 */
	gyr_act.x=(gyr_act.x)/180.0f*PI;
	gyr_act.y=(gyr_act.y)/180.0f*PI;
	gyr_act.z=(gyr_act.z)/180.0f*PI;
	
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

	#ifdef DEBUG_ENABLE
	
	USART_OUT_F(result_angle.z);
	#endif 

	return wait_second;
}
/**
  * @brief  二次矫正零点漂移
  * @param  w:  输入的角速度
  * @retval 矫正后的角速度
  */
uint8_t adjustVDoff(three_axis *w)
{
	/* 时间控制值 */
	/*count不会清零*/
	static uint32_t count=0;
	static three_axis adjust={0,0,0};
	
	/* 时间控制，控制周期为200Hz，200次1秒 */
	if(count<=200)
	{
	 count++;
	 adjust.x=adjust.x+w->x;
	 adjust.y=adjust.y+w->y;
	 adjust.z=adjust.z+w->z;
		
	 /* 矫正完成前控制输出的角速度为0 */
	 w->x=0;
	 w->y=0;
	 w->z=0;
		
	 return 0;
	}
	else
	{
		/* 得到矫正后的角速度 */
		w->x=w->x-adjust.x/200.0f;
		w->y=w->y-adjust.y/200.0f;
		w->z=w->z-adjust.z/200.0f;
	}
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
/**
  * @brief  将四元数转换为欧拉角
  * @param  quaternion: 需要转换的四元数
  * @retval 四元数对应的欧拉角
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
  * @brief  将欧拉角转换为四元数
  * @param  quaternion: 需要转换的欧拉角
  * @retval 四元数对应的四元数
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
  * @brief  针对四元数与角速度的积分  二阶龙格库塔法
  * @param  quaternion: 原始的姿态
  * @param  data      : 设备的角速度
  * @retval 积分完后的姿态
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
	/*#define dT 					   0.005f           //积分的步长5ms*/
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
	
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_now;        //本次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.01;       //系统噪声
//	static double R=0.0002;      //测量噪声     
	static double R=0.012;      //测量噪声             
	
	static float IAE_st[50];    //记录的新息
	double Cr=0;                //新息的方差
	
	/* 获得来自用户的校正值，如果用户不矫正，则使用0.012 */
	//R=Get_R_Zaxis();
   
	predict=act_value;
	
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=ordata-predict;
	
	/* 新息的方差计算 */
	Cr=0;
	for(i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/49.0f;
	
	
	/* 预测本次的预测误差 */
	P_mid=P_last+Q;
	
	/* 计算系数，求得输出值 */
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(ordata-predict);
	
	/* 更新预测误差 */
	P_now=(1-Kk)*P_mid;
	
	/* 计算并调整系统噪声 */
	Q=Kk*Kk*Cr;
	
	P_last=P_now;

	/* 为提高滤波器的响应速度，减小滞后而设下的阈值 */
	if(Kk>0.5)
		act_value=ordata;
	
	return act_value;
}

float KalmanFilterT(float ordata)
{
	uint8_t i;
	
  static double act_value=0;  //实际值
  double predict;             //预测值
  
	static double P_last=0.1;   //上一次的预测误差
	static double P_now;        //本次的预测误差
	static double P_mid;        //对预测误差的预测
	static double Kk;           //滤波增益系数
	
	static double Q=0.8;       //系统噪声
	static double R=0.973067;      //测量噪声          
	
	static float IAE_st[50];    //记录的新息
	double Cr=0;                //新息的方差
	
	/* 获得来自用户的校正值，如果用户不矫正，则使用0.012 */
	R=Get_R_Zaxis();
   
	predict=act_value;
	
	//相当于数组的左移
	memcpy(IAE_st,IAE_st+1,196);
	IAE_st[49]=ordata-predict;
	
	/* 新息的方差计算 */
	Cr=0;
	for(i=0;i<50;i++)
	{
		Cr=Cr+IAE_st[i]*IAE_st[i];
	}
	Cr=Cr/49.0;		//样本方差（不知道对不对，还要看原公式）
	
	/* 预测本次的预测误差 */
	P_mid=P_last+Q;
	
	/* 计算系数，求得输出值 */
	Kk=P_mid/(P_mid+R);
	act_value=predict+Kk*(ordata-predict);
	
	/* 更新预测误差 */
	P_now=(1-Kk)*P_mid;
	
	/* 计算并调整系统噪声 */
	Q=Kk*Kk*Cr;
	
	P_last=P_now;

	/* 为提高滤波器的响应速度，减小滞后而设下的阈值 */
	if(Kk>0.5)
		act_value=ordata;
	
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
	USART_OUT_F(gyr_act.z);
	USART_OUT_F(result_angle.z);
	USART_OUT(USART1,"\r\n");
	#endif
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
