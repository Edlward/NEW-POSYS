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
#include "kalman.h"
#include "self_math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

extern AllPara_t allPara;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/**
* @brief  更新角度值
*
* @param[in] gyrData,输入陀螺仪的加速度值
*            magData,输入磁力计的值
*            accData,输入加速度的值
* @retval 初始化完成的标志位
*/

#ifdef AUTOCAR
#define TIME_STATIC					(20)
#define TIME_STATIC_REAL		(TIME_STATIC-(2))
#else
#define TIME_STATIC					(3)
#define TIME_STATIC_REAL		(TIME_STATIC-(2))
#endif


#define STATIC_MAX_NUM	1000
#define STATIC_MIN_NUM	600

double lowpass=0.0;
static float stdCr[AXIS_NUMBER]={0.f};        
static float mean[AXIS_NUMBER]={0.f};
int RoughHandle(void)
{
  static int ignore=0;
	static int isCalCrOk=0;
	
  allPara.GYRO_Real[0]=(allPara.sDta.GYRO_Aver[0]);
  allPara.GYRO_Real[1]=(allPara.sDta.GYRO_Aver[1]);
  allPara.GYRO_Real[2]=(allPara.sDta.GYRO_Aver[2]);
	
	lowpass=LowPassFilterGyro(allPara.sDta.GYRO_Aver[2]);
	if(allPara.resetFlag)
		ignore=TIME_STATIC_REAL*200+1;
	
	ignore++;
	
	if(ignore>200*3&&!isCalCrOk)
	{
		if(CalculateRealCrAndMean(stdCr,mean)){}
			//isCalCrOk=1;
	}
	else if(isCalCrOk==1)
	{
		#ifdef TEST_SUMMER
		int axis=2;
		if(!(fabs(allPara.sDta.GYRO_Aver[axis]-mean[axis])<stdCr[axis]*3))
		{
//			USART_OUT_F(allPara.sDta.GYRO_Aver[axis]);
//			USART_OUT_F(mean[axis]);
//			USART_OUT_F(stdCr[axis]);
//			USART_OUT(USART3,"NNNNNN");
//			USART_Enter();
		}
		#endif
	}
  if(ignore>(TIME_STATIC_REAL)*200){
		if(UpdateBais()||allPara.resetFlag)
		{
			ignore=(TIME_STATIC_REAL)*200+5;
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

void updateAngle(void)
{	
	double w[3]={0.0};
	
	w[0]=allPara.GYRO_Real[0];
	w[1]=allPara.GYRO_Real[1];
	w[2]=allPara.GYRO_Real[2];
	
  if(fabs(w[0])<0.3f)//单位 °/s
    w[0]=0.f;
  if(fabs(w[1])<0.3f)//单位 °/s
    w[1]=0.f;

	#ifdef AUTOCAR
	if((allPara.sDta.flag&STATIC_FORCE)||(abs(allPara.sDta.vell[0])<=1&&abs(allPara.sDta.vell[1])<=1&&fabs(w[2])<0.20))//单位 °/s
    w[2]=0.f;
	#else
	if((allPara.sDta.flag&STATIC_FORCE)||(fabs(FilterVell(allPara.sDta.vell[1]))<=20.f))//||(fabs(w[2])<0.3f))
    w[2]=0.f;
	#endif
	
	allPara.sDta.Result_Angle[2]+=w[2]*0.005;
	if(allPara.sDta.Result_Angle[2]>180.0)
		allPara.sDta.Result_Angle[2]-=360.0;
	else if(allPara.sDta.Result_Angle[2]<-180.0)
		allPara.sDta.Result_Angle[2]+=360.0;
	
}


uint8_t UpdateBais(void)
{  
	static int index=0;
  static double data[AXIS_NUMBER][STATIC_MAX_NUM]={0.f};    	//数据列
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
	
	/*条件一	比赛还没开始*/
	if((!(allPara.sDta.flag&START_COMPETE))\
		/*条件二	收集的数还很少*/
		||(cnt<=STATIC_MIN_NUM))
//		/*条件三	强制进入静止状态，并且条件还好（后续有判断）*/
//		||(allPara.sDta.flag&STATIC_FORCE))
	{
		index++;
		/*如果已存数据小于等于最大长度，顺序一个一个存储*/
		if(index<=STATIC_MAX_NUM)
		{
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				/*如果满足3*std法则*/
				if(fabs(allPara.sDta.GYRO_Aver[axis]-mean[axis])<stdCr[axis]*3)
					data[axis][index-1]=allPara.sDta.GYRO_Aver[axis];
				else
				{
					if(axis==2)
					{
						index--;
						cnt--;
					}
				}
//				if(axis==2)
//					data[axis][index-1]=lowpass;
			}
		}
		//如果超出最大值，减去第一个值，加入
		else if(index>STATIC_MAX_NUM)
		{
			index=STATIC_MAX_NUM;
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				//数组前移一位
				if(fabs(allPara.sDta.GYRO_Aver[axis])<4.f&&!isnan(allPara.sDta.GYRO_Aver[axis]))
				{
					/*如果满足3*std法则*/
					if(fabs(allPara.sDta.GYRO_Aver[axis]-mean[axis])<stdCr[axis]*3)
					{
						for(int i=0;i<STATIC_MAX_NUM-1;i++)
							data[axis][i]=data[axis][i+1];
						data[axis][STATIC_MAX_NUM-1]=allPara.sDta.GYRO_Aver[axis];
					}
//					if(axis==2)
//						data[axis][STATIC_MAX_NUM-1]=lowpass;
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
			for(int axis=0;axis<AXIS_NUMBER;axis++)
			{
				for(int i=0;i<index-1;i++)
				{
					sum[axis]=sum[axis]+data[axis][i];
				}
				if((fabs(sum[axis]/(index-1)-allPara.sDta.GYRO_Bais[axis])<0.009&&allPara.sDta.GYRO_Bais[axis]!=0.0)||(allPara.sDta.GYRO_Bais[axis]==0.0))
					allPara.sDta.GYRO_Bais[axis]=sum[axis]/(index-1);
			}
		}
		for(int axis=0;axis<AXIS_NUMBER;axis++)
			for(int i=0;i<STATIC_MAX_NUM;i++)
				data[axis][i]=0.0;
		index=0;
	}
	
	return returnValue;
}




void SetAngle(float angle){
	double euler[3];
	euler[0]=0.0f;
	euler[1]=0.0f;
	euler[2]=angle/180.f*PI;
	Euler_to_Quaternion(euler,allPara.sDta.quarternion);
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
