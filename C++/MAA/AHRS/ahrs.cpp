#include "ahrs.h"
#include "calculateAttitude.h"
#include "timer.h"
#include "ADXRS453.h"
#include "ICM20602.h"
#include "LSM303AGR.h"
#include "usart.h"
#include "action_matrix.h"
#include "attitudeAngle.h"
#include "adaptiveUnscentedKalmanFilter.h"
#include "user.h"
#include "signalProcess.h"
#include "pos.h"

#define AVERAGE_TIME 				(10)
static const uint32_t calculateTime=AVERAGE_TIME*PERIOD;
static threeAxis dataICM={0,0,0};
static threeAxis eulerAngle;
static threeAxis dataICM20602[ICM_GYRO_NUM]={0.f,0.f,0.f};
static float tempICM20602[ICM_GYRO_NUM]={ 0.0f };


void AHRS_Init(void)
{
		//one second, one array element
	threeAxis newDataICM20602[ICM_GYRO_NUM][calculateTime/PERIOD];
	float     newTempICM20602[ICM_GYRO_NUM][calculateTime/PERIOD];
	
	if(getICM20602_Gyro())
	{
		for(int i=0;i<getICM20602_Gyro()[0]->getInstanceNum();i++)
		{
			dataICM20602[i]=0.f;
			tempICM20602[i]=0.f;
		}
	}
	
	uint16_t count=0;
	uint16_t countSec=0;
	
	while((!DEVICE_IS_RUNNING)||(countSec<=(calculateTime/100)))
	{
		while(!getTimeFlag());
		
		if(getICM20602_Gyro())
		{
			for(int i=0;i<getICM20602_Gyro()[0]->getInstanceNum();i++)
			{
				dataICM20602[i]=getICM20602_Gyro()[i]->getData();
				tempICM20602[i]=getICM20602_Gyro()[i]->temp;
			}
		}
		
		count++;
		//if time pass by one second
		if(count%PERIOD==0)
		{
			countSec++;
			/*shift data right one unit*/
			shiftRightData(newDataICM20602[0],calculateTime/PERIOD);
			shiftRightData(newTempICM20602[0],calculateTime/PERIOD);
			if(getICM20602_Gyro())
			{
				for(int i=0;i<getICM20602_Gyro()[0]->getInstanceNum();i++)
				{
					dataICM20602[i]=getICM20602_Gyro()[i]->getData();
					tempICM20602[i]=getICM20602_Gyro()[i]->temp;
					/*add new value(mean of period)*/
					newDataICM20602[i][0]=dataICM20602[i]/PERIOD;
					newTempICM20602[i][0]=tempICM20602[i]/PERIOD;
					
					dataICM20602[i]=0;
					tempICM20602[i]=0;
				}
			}	
		}
	}
	
	dataICM20602[0]=meanData(newDataICM20602[0],calculateTime/100);
	tempICM20602[0]=meanData(newTempICM20602[0],calculateTime/100);
}


void AHRS_Update(void)
{
	
}
