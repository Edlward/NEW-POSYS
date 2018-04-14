#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "stdint.h"	
#include "timer.h"
#ifdef __cplusplus
}
 #endif

#define ICM_HeatingPower(value) 	TIM_SetCompare2(TIM5,(uint32_t)((double)value/100.0*1000));

#define MAX_TEMPERATURE			48.f

class TemperatureControl
{
	private:
		/*heating init, and make sure tempAim*/
		float tempAim=0.f;
		int time=0;
		
		/*temperature control parameters*/
		float K_p = 106.0f;
		float K_i = 0.035f;
		float err=0.f;
	  float integral=0.f;
	  float ctr=0.f;
		int countForHeatSafe=0;
	
		/*lowpass Filter parameters*/  
		float K_x=0.002f; //滤波系数
		uint32_t num_x=0;//滤波计数器
		uint8_t new_flag_x=0;//本次数据变化方向
		uint8_t old_flag_x=0;
		float valueLast=0.0f;

	public:
		//is temp init succeeded
		int tempInitSuces=0;
		//default constructor
		TemperatureControl()=default;
		void temp_pid_ctr(float temp);
		int HeatingInit(float temp);
		/*functions*/
		float LowPassFilter(float temp);
};


#endif

