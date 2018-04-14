/**
  ******************************************************************************
  * @file    Temperature.h
  * @author  Qiao Zhijian 
  * @version V1.0
  * @date    2017.3.13
  * @brief   This file contains the headers of usart.cpp
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#include "temperature.h"
#include "arm_math.h"

int TemperatureControl::HeatingInit(float temp)
{
	time++;

	if(time==10)
	{
		tempAim=temp+5.f;
		if(tempAim>MAX_TEMPERATURE)
		{
			tempAim=MAX_TEMPERATURE;
		}
		tempInitSuces=1;
		return 1;
	}
	return 0;
}

void TemperatureControl::temp_pid_ctr(float temp)
{
	/*eror*/
	err=tempAim-temp;
	/*integral of err*/
	integral=integral+err;

	ctr=K_p*err+K_i*integral;
			
	/*determine upper limit*/
	if(ctr>100)
	{
		ctr=100;
	}else if(ctr<0)
	{
		ctr=0;
	}
	
	/*prevent hyperthermia*/
	if(temp>(MAX_TEMPERATURE+2.f))
		ctr=0;
	
	//prevent gyroscope from excessive heating 
	if(fabs(temp-tempAim)>4.f)
		countForHeatSafe++;
	else
		countForHeatSafe=0;
	/*if excessive heating for a long time of 4s, make ctr equal to 20 compulsively */
	if(countForHeatSafe>2*200)
		ctr=20;
	
	ICM_HeatingPower(ctr);
}


#define Threshold_1 		 0.02f		
#define Threshold_2			 10				
float TemperatureControl::LowPassFilter(float temp)
{

	if(valueLast==0.0f)
		valueLast=temp;
	
	//judge the direction of changing
  if((temp-valueLast)>0.0f)
  {
    new_flag_x=1;
  }
  if((temp-valueLast)<0.0f)
  {
    new_flag_x=0;
  }
  
	//if the same as the last trend 
  if(new_flag_x==old_flag_x) 
  {
    num_x=num_x+1;
		//continued growing
    if(fabs(temp-valueLast)>Threshold_1)
    {
      num_x=num_x+5;   
    }
		
		if(num_x>Threshold_2) 
		{
			//add K_x to enhance the following effect 
			K_x=K_x+0.07f; 
			if(K_x>1.0f) 
				K_x=1.0f; 
			num_x=0;
		}
	}
  else 
  {
    num_x=0;
		K_x=K_x-0.07f;
		if(K_x<0.01f)
			K_x=0.01f; 
  }
	
  valueLast=(1-K_x)*valueLast+K_x*temp;
  old_flag_x=new_flag_x;
  
  return valueLast;
  
}

