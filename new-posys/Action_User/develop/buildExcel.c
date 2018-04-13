/**
******************************************************************************
* @file     
* @author  lxy and Summer
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

extern AllPara_t allPara;

/*
chartW排序方式  
陀螺仪1（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））
陀螺仪2（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））
陀螺仪3（X轴（TEMP_SAMPLE_NUMBER个结果）Y轴（TEMP_SAMPLE_NUMBER个结果）Z轴（TEMP_SAMPLE_NUMBER个结果））

chart+陀螺仪序号（0-(GYRO_NUMBER-1）*AXIS_NUMBER*+轴号（0-(AXIS_NUMBER-1）*TEMP_SAMPLE_NUMBER+结果号（0-(TEMP_SAMPLE_NUMBER-1)）
*/


/*
chartMode方式  
陀螺仪1（X轴  Y轴  Z轴）
陀螺仪2（X轴  Y轴  Z轴）
陀螺仪3（X轴  Y轴  Z轴）

chart+陀螺仪序号（0-(GYRO_NUMBER-1)）*AXIS_NUMBER+轴号（0-(AXIS_NUMBER-1））
*/

void TempTablePrintf(void)
{
  
}

void PrintChartMode(void)
{

}

void PrintVarXYZ(void)
{

}

void PrintMinValue(void)
{

}

void PrintScaleMode(void)
{

}

void PrintchartSelect(void)
{
 
}

int CalculateCrAndMean(float stdCr[GYRO_NUMBER][AXIS_NUMBER],float mean[GYRO_NUMBER][AXIS_NUMBER]){
  static float IAE_st[GYRO_NUMBER][AXIS_NUMBER][200]={0.f};    //记录的新息
  static float data[GYRO_NUMBER][AXIS_NUMBER][200]={0.f};    	//数据列
  static int ignore=0;
  ignore++;
  
  int gyro=0;
  int axis=0;
  
  /* 新息的方差计算 */
  for(gyro=0;gyro<GYRO_NUMBER;gyro++)
    for(axis=0;axis<AXIS_NUMBER;axis++)
    {
      mean[gyro][axis]=mean[gyro][axis]-data[gyro][axis][0]/200;
      memcpy(data[gyro][axis],data[gyro][axis]+1,796);
      data[gyro][axis][199]=allPara.GYROWithoutRemoveDrift[gyro][axis];
      memcpy(IAE_st[gyro][axis],IAE_st[gyro][axis]+1,796);
      mean[gyro][axis]=mean[gyro][axis]+data[gyro][axis][199]/200;
      IAE_st[gyro][axis][199]=allPara.GYROWithoutRemoveDrift[gyro][axis]-mean[gyro][axis];
    }
  
  if(ignore<400)
    return 0;
  else
    ignore=400;
  
  for(gyro=0;gyro<GYRO_NUMBER;gyro++)
    for(axis=0;axis<AXIS_NUMBER;axis++){
      stdCr[gyro][axis]=0;
      for(int i=0;i<200;i++)
      {
        stdCr[gyro][axis]=stdCr[gyro][axis]+IAE_st[gyro][axis][i]*IAE_st[gyro][axis][i];
      }
      stdCr[gyro][axis]=__sqrtf(stdCr[gyro][axis]/200.0f);
    }
  
  return 1;
}

int WaitForSlowDrift(void)
{
  static uint32_t time=0;
 	time++;
 	if(time<200*60*30)
 		return 0;
 	else
  {
    time=200*60*31;
    return 1;
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
int UpdateVDoffTable(void)
{
  static uint32_t temp_count[GYRO_NUMBER][TempTable_NUMBER]={0u};
  static long double temp_w[GYRO_NUMBER][AXIS_NUMBER][TempTable_NUMBER]={0.0};
  long double paraXY[GYRO_NUMBER][AXIS_NUMBER]={0.0};
  long double paraX[GYRO_NUMBER]    = {0.0};
  long double paraY[GYRO_NUMBER][AXIS_NUMBER] ={0.0};
  long double paraX2[GYRO_NUMBER]   = {0.0};
  static float stdCr[GYRO_NUMBER][AXIS_NUMBER]={0.f};                //新息的标准差
  static float mean[GYRO_NUMBER][AXIS_NUMBER]={0.f};
  
  int index=0;
  int gyro=0;
  int axis=0;
  
  if(!WaitForSlowDrift())
    return 0;
  
  /*三σ法则*/
  if(!CalculateCrAndMean(stdCr,mean))
    return 0;
  
  for(gyro=0;gyro<GYRO_NUMBER;gyro++)
    for(axis=0;axis<AXIS_NUMBER;axis++)
    {
      if(fabs(allPara.GYROWithoutRemoveDrift[gyro][axis]-mean[gyro][axis])>stdCr[gyro][axis]*5)
        return 0;
    }
  
  for(gyro=0;gyro<GYRO_NUMBER;gyro++)
  {
    if((double)(allPara.GYRO_Temperature[gyro])>=TempTable_min&&(double)(allPara.GYRO_Temperature[gyro])<TempTable_max-0.0006){
      /*确定温度索引号,如果不减0.006可能会出现index=200的情况*/
      index=roundf(((double)(allPara.GYRO_Temperature[gyro])-TempTable_min)*1000.0);
      if(temp_count[gyro][index]>0){
        /*求这一个温度上的角速度和*/
        for(axis=0;axis<AXIS_NUMBER;axis++)
          temp_w[gyro][axis][index]+=allPara.GYROWithoutRemoveDrift[gyro][axis];
      }
      else{
        /*初始值*/
        for(axis=0;axis<AXIS_NUMBER;axis++)
          temp_w[gyro][axis][index]=allPara.GYROWithoutRemoveDrift[gyro][axis];
        temp_count[gyro][index]=0;
      }
      temp_count[gyro][index]++;
    }
  }
  
	static int time = 0;
	time++;
	if(time==200)
	{
		time=0;
		USART_OUT_F(allPara.GYRO_Temperature[0]);
		USART_OUT_F(stdCr[0][2]);
		USART_OUT_F(allPara.GYRO_Temperature[1]);
		USART_OUT_F(stdCr[1][2]);
		USART_OUT_F(allPara.GYRO_Temperature[2]);
		USART_OUT_F(stdCr[2][2]);
		USART_Enter();
	}
	
  for(gyro=0;gyro<GYRO_NUMBER;gyro++)
  {			
    if(TempErgodic(gyro,0)==3)
    {
      int32_t sum[GYRO_NUMBER]={ 0 };
      USART_OUT(USART3,"\r\nfinish %d \r\n ",gyro);
      for(int i=0;i<TempTable_NUMBER;i++)
      {
        if(temp_count[gyro][i]>0){
          long double chartTemperature=(long double)(TempTable_min*1000.0+i)/1000.0;
          paraX[gyro]=paraX[gyro]+temp_count[gyro][i]*chartTemperature;
          paraX2[gyro]=paraX2[gyro] + temp_count[gyro][i]*chartTemperature*chartTemperature;
          sum[gyro]=sum[gyro]+temp_count[gyro][i];
          for(axis=0;axis<AXIS_NUMBER;axis++)
          {
            paraXY[gyro][axis]=paraXY[gyro][axis]+temp_w[gyro][axis][i]*chartTemperature;
            paraY[gyro][axis]=paraY[gyro][axis]+temp_w[gyro][axis][i];					
            temp_w[gyro][axis][i]=temp_w[gyro][axis][i]/temp_count[gyro][i];
          }
        }
				else{
          temp_count[gyro][i]=0;
          temp_w[gyro][0][i]=0.f;
          temp_w[gyro][1][i]=0.f;
          temp_w[gyro][2][i]=0.f;
        }
      }
      for(axis=0;axis<AXIS_NUMBER;axis++)
      {
        (sum[gyro]*paraXY[gyro][axis]-paraX[gyro]*paraY[gyro][axis])/(sum[gyro]*paraX2[gyro]-paraX[gyro]*paraX[gyro]);
      }
      TempErgodic(gyro,1);
      USART_OUT(USART3,"Flash Update end\r\n");
      TempTablePrintf();
      
      for(int i=0;i<roundf((TempTable_max*1000.0-TempTable_min*1000.0));i++)
      {	
        paraX[gyro]=0.0;
        paraX2[gyro]=0.0;
        temp_count[gyro][i]=0;
        for(axis=0;axis<AXIS_NUMBER;axis++)
        {
          temp_w[gyro][axis][i]=0.f;
          paraXY[gyro][axis]=0.0;
          paraY[gyro][axis]=0.0;
        }
        
      }
    }
  }
  return 1;
}


void Hex_To_Str(uint8_t * pHex,char * s,float num)
{
  char        hex[] = "0123456789ABCDEF";
  char        *pStr = s;
  for (uint8_t i = 0; i < (int)(num/2.f+0.5f); i++)//(int)(x+0.5f)是把x四舍五入的意思
  {
    
    /*
    1.*pStr++右结合,并且*索引的是没有++之前的地址
    2.f.移位不会改变指针指向的那个空间的值
    3.对指针指向空间的移位也不会改变指针的指向
    */
    if (((num<((int)(num / 2.f + 0.5f))*2.f)&&i>0)|| (num==((int)(num / 2.f + 0.5f)) * 2.f))
      *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) >> 4];
    *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) & 0x0F];
  }
}



/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
