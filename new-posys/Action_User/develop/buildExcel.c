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

extern flashData_t flashData;
extern AllPara_t allPara;
void TempTablePrintf(void)
{
  Flash_Read(GetFlashArr(),TempTable_Num);
  
  USART_OUT(USART1,"\r\nchartWX:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*(flashData.chartWX+i));
  
  USART_OUT(USART1,"\r\nchartWY:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*((flashData.chartWY)+i));
  
  USART_OUT(USART1,"\r\nchartWZ:\r\n");
  for(int i=0;i<5;i++)
    USART_OUT_F(*((flashData.chartWZ)+i));
  
  USART_OUT(USART1,"\r\nchartMode:\r\n%d",*(flashData.chartMode));
  USART_OUT(USART1,"\r\nchartSelect:\r\n%d\t%d\t%d\t%d\t%d\r\n",*(flashData.chartSelect),*((flashData.chartSelect)+1),*((flashData.chartSelect)+2),*((flashData.chartSelect)+3),*((flashData.chartSelect)+4));
  USART_OUT(USART1,"(flashData.scaleMode): %d\r\n",*(flashData.scaleMode));
  USART_OUT(USART1,"(flashData.minValue):\r\n");
  USART_OUT_F(*((flashData.minValue)));
	USART_OUT(USART1,"\r\nvarXYZ:  ");
  USART_OUT_F(*((flashData.varXYZ)+0));
  USART_OUT_F(*((flashData.varXYZ)+1));
  USART_OUT_F(*((flashData.varXYZ)+2));
	USART_Enter();
  
}

int CalculateCrAndMean(float stdCr[GYRO_NUMBER][AXIS_NUMBER],float mean[GYRO_NUMBER][AXIS_NUMBER]){
	static float IAE_st[GYRO_NUMBER][AXIS_NUMBER][200]={0.f};    //记录的新息
	static float data[GYRO_NUMBER][AXIS_NUMBER][200]={0.f};    	//数据列
	static int ignore=0;
	ignore++;
  /* 数据列表 */
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		for(int axis=0;axis<AXIS_NUMBER;axis++)
			memcpy(data[gyro][axis],data[gyro][axis]+1,796);
		
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		for(int axis=0;axis<AXIS_NUMBER;axis++)
			data[gyro][axis][199]=allPara.GYROWithoutRemoveDrift[gyro][axis];
	
  /* 新息的方差计算 */
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		for(int axis=0;axis<AXIS_NUMBER;axis++)
		{
			memcpy(IAE_st[gyro][axis],IAE_st[gyro][axis]+1,796);
			mean[gyro][axis]=mean[gyro][axis]-data[gyro][axis][199]/200+data[gyro][axis][199]/200;
		}
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		for(int axis=0;axis<AXIS_NUMBER;axis++)
			IAE_st[gyro][axis][199]=allPara.GYROWithoutRemoveDrift[gyro][axis]-mean[gyro][axis];
	
  if(ignore<400)
		return 0;
	else
		ignore=400;
	
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
		for(int axis=0;axis<AXIS_NUMBER;axis++){
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
	static uint32_t temp_count[GYRO_NUMBER][(int)(TempTable_max-TempTable_min)*10]={0u};
	static long double temp_w[GYRO_NUMBER][AXIS_NUMBER][(int)(TempTable_max-TempTable_min)*10]={0.0};
  long double paraXY[GYRO_NUMBER][AXIS_NUMBER]={0.0};
  long double paraX[GYRO_NUMBER]    = {0.0};
  long double paraY[GYRO_NUMBER][AXIS_NUMBER] ={0.0};
  long double paraX2[GYRO_NUMBER]   = {0.0};
  int index=0;
  static float stdCr[GYRO_NUMBER][AXIS_NUMBER]={0.f};                //新息的标准差
  static float mean[GYRO_NUMBER][AXIS_NUMBER]={0.f};
	
	if(!WaitForSlowDrift())
		return 0;
	
	/*三σ法则*/
	if(!CalculateCrAndMean(stdCr,mean))
		return 0;
	
	if((fabs(allPara.GYRO_Aver[0]-mean[0])>stdCr[0]*5)||(fabs(allPara.GYRO_Aver[1]-mean[1])>stdCr[1]*5)||(fabs(allPara.GYRO_Aver[2]-mean[2])>stdCr[2]*5))
	{
		return 0;
	}
	else
	{	
	}		
	
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		if((double)(allPara.GYRO_Temperature[gyro])>=TempTable_min&&(double)(allPara.GYRO_Temperature[gyro])<TempTable_max-0.006){
			/*确定温度索引号,如果不减0.006可能会出现index=200的情况*/
			index=roundf(((double)(allPara.GYRO_Temperature[gyro])-TempTable_min)*100.0);
			if(temp_count[gyro][index]>0){
				/*求这一个温度上的角速度和*/
				for(int axis=0;axis<AXIS_NUMBER;axis++)
					temp_w[gyro][axis][index]+=allPara.GYROWithoutRemoveDrift[gyro][axis];
			}
			else{
				/*初始值*/
				for(int axis=0;axis<AXIS_NUMBER;axis++)
					temp_w[gyro][axis][index]=allPara.GYROWithoutRemoveDrift[gyro][axis];
				temp_count[gyro][index]=0;
			}
				temp_count[gyro][index]++;
		}
	}
	
	
  if(TempErgodic(0)==3){
		int32_t sum=0;
		USART_OUT(USART1,"\r\nfinish\r\n");
		for(int i=0;i<(TempTable_max-TempTable_min)*10;i++)
		{
			if(temp_count[i]>500){
				paraX=paraX+temp_count[i]*((long double)(TempTable_min+i/10.0)/100.0);
				paraX2  =paraX2 + temp_count[i]*(long double)((TempTable_min+i/10.0)/100.0)*(long double)((TempTable_min+i/10.0)/100.0);
				paraXY[0]=paraXY[0]+temp_w[0][i]*(long double)((TempTable_min+i/10.0)/100.0);
				paraXY[1]=paraXY[1]+temp_w[1][i]*(long double)((TempTable_min+i/10.0)/100.0);
				paraXY[2]=paraXY[2]+temp_w[2][i]*(long double)((TempTable_min+i/10.0)/100.0);
				paraY[0]=paraY[0]+temp_w[0][i];
				paraY[1]=paraY[1]+temp_w[1][i];
				paraY[2]=paraY[2]+temp_w[2][i];
				sum=sum+temp_count[i];
				
				temp_w[0][i]=temp_w[0][i]/temp_count[i];
				temp_w[1][i]=temp_w[1][i]/temp_count[i];
				temp_w[2][i]=temp_w[2][i]/temp_count[i];
				USART_OUT_F(TempTable_min+i/10.0);
				USART_OUT_F(temp_w[0][i]);
				USART_OUT_F(temp_w[1][i]);
				USART_OUT_F(temp_w[2][i]);
				USART_Enter();	
				delay_us(200);
			}else{
				temp_count[i]=0;
				temp_w[0][i]=0.f;
				temp_w[1][i]=0.f;
				temp_w[2][i]=0.f;
//				USART_OUT_F(TempTable_min+i/10.0);
//				USART_Enter();
			}
		}
    for(int i=4;i>0;i--){
      *((flashData.chartWX)+i)=*((flashData.chartWX)+i-1);
      *((flashData.chartWY)+i)=*((flashData.chartWY)+i-1);
      *((flashData.chartWZ)+i)=*((flashData.chartWZ)+i-1);
    }
		TempErgodic(1);
		//2*60/0.005=24000
    *(flashData.chartWX)=(sum*paraXY[0]-paraX*paraY[0])/(sum*paraX2-paraX*paraX);
    *(flashData.chartWY)=(sum*paraXY[1]-paraX*paraY[1])/(sum*paraX2-paraX*paraX);
    *(flashData.chartWZ)=(sum*paraXY[2]-paraX*paraY[2])/(sum*paraX2-paraX*paraX);
    Flash_Write(GetFlashArr(),TempTable_Num);
    USART_OUT(USART1,"Flash Update end\r\n");
    TempTablePrintf();
		{
			for(int i=0;i<3;i++){
				paraXY[i]=0.0;
				paraY[i]=0.0;
				paraX=0.0;
				paraX2=0.0;
			}
			for(int i=0;i<(int)(TempTable_max-TempTable_min)*10;i++)
			{
				temp_count[i]=0;
				temp_w[0][i]=0.f;
				temp_w[1][i]=0.f;
				temp_w[2][i]=0.f;
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
