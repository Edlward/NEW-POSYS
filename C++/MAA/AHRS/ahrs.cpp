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
static threeAxis dataICM20602={0.f,0.f,0.f};
static float tempICM20602= 0.0f ;


void AHRS_Init(void)
{
		//one second, one array element
	threeAxis newDataICM20602[calculateTime/PERIOD];
	float     newTempICM20602[calculateTime/PERIOD];
	
	dataICM20602=0.f;
	tempICM20602=0.f;
	
	uint16_t count=0;
	uint16_t countSec=0;
	
	while((!DEVICE_IS_RUNNING)||(countSec<=(calculateTime/100)))
	{
		while(!getTimeFlag());
		
		dataICM20602=getICM20602_Gyro().getData();
		tempICM20602=getICM20602_Gyro().temp;

		count++;
		//if time pass by one second
		if(count%PERIOD==0)
		{
			countSec++;
			/*shift data right one unit*/
			shiftRightData(newDataICM20602,calculateTime/PERIOD);
			shiftRightData(newTempICM20602,calculateTime/PERIOD);

			dataICM20602=getICM20602_Gyro().getData();
			tempICM20602=getICM20602_Gyro().temp;
			
			/*add new value(mean of period)*/
			newDataICM20602[0]=dataICM20602/PERIOD;
			newTempICM20602[0]=tempICM20602/PERIOD;
					
			dataICM20602=0;
			tempICM20602=0;
		}
	}
	
	dataICM20602=meanData(newDataICM20602,calculateTime/100);
	tempICM20602=meanData(newTempICM20602,calculateTime/100);
}


void AHRS_Update(void)
{
	
}
