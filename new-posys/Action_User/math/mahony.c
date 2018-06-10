#include "mahony.h"
#include "config.h"
#include "self_math.h"
#include "quarternion.h"

extern AllPara_t allPara;

//mahony滤波器Kp系数初始值
float KpInit=10;
//mahony滤波器Ki系数初始值
float KiInit=1;

void InitQuarternion(void)
{
	//quaternionMahony=Euler_to_Quaternion([atan2d(ay(1),az(1)) asind(-ax(1)) 0]);
	double initangle[3]={0.0,0.0,0.0};
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		float X_G,Y_G,Z_G;
		float tempSum=sqrt(allPara.ACC_Raw[gyro][0]*allPara.ACC_Raw[gyro][0]+allPara.ACC_Raw[gyro][1]*allPara.ACC_Raw[gyro][1]+allPara.ACC_Raw[gyro][2]*allPara.ACC_Raw[gyro][2]);
		X_G=allPara.ACC_Raw[gyro][0]/tempSum;
		Y_G=allPara.ACC_Raw[gyro][1]/tempSum;
		Z_G=allPara.ACC_Raw[gyro][2]/tempSum;
		/*初始坐标为0,0,g,然后可以通过坐标变换公式轻易推导*/
		allPara.ACC_Angle[gyro][0]= safe_atan2(Y_G,Z_G);
		allPara.ACC_Angle[gyro][1]= safe_asin(-X_G);
	}
	
	initangle[0]=allPara.ACC_Angle[0][0];
	initangle[1]=allPara.ACC_Angle[0][1];
	initangle[2]=0.0;
	Euler_to_Quaternion(initangle,allPara.sDta.quarternion);
	//初始化加速度开始的时候的总和
	allPara.sDta.mahony_t.accSumStatic=sqrt(allPara.ACC_Raw[0][0]*allPara.ACC_Raw[0][0]+allPara.ACC_Raw[0][1]*allPara.ACC_Raw[0][1]+allPara.ACC_Raw[0][2]*allPara.ACC_Raw[0][2]);
}



void MahonyFilter(void)
{
	//采用的
	float Kp=0.f;
	float Ki=0.f;
	double ex,ey,ez=0.0;
	double vx,vy,vz=0.0;
	double ax,ay,az=0.0;
	double w[3]={0.0};
	double gyroMahony[3]={0.0};
	float accSum[GYRO_NUMBER]={0.f};
	double q[4]={allPara.sDta.quarternion[0],allPara.sDta.quarternion[1],allPara.sDta.quarternion[2],allPara.sDta.quarternion[3]};

	//对角速度进行处理
	for(int i=0;i<3;i++)
	{
		w[i]=allPara.GYRO_Real[i];
		if(i!=2)
		{
			if(fabs(w[i])<0.1f)//单位 °/s
				w[i]=0.f;
		}
		else
		{
			#ifdef AUTOCAR
			if((allPara.sDta.flag&STATIC_FORCE))//单位 °/s
				w[i]=0.f;
			#else
			if((allPara.sDta.flag&STATIC_FORCE)||(fabs(FilterVell(allPara.vell[1]))<=20.f)))
				w[i]=0.f;
			#endif
		}
	}
	
	//对加速度进行处理
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		float X_G,Y_G,Z_G;
		accSum[gyro]=sqrt(allPara.ACC_Raw[gyro][0]*allPara.ACC_Raw[gyro][0]+allPara.ACC_Raw[gyro][1]*allPara.ACC_Raw[gyro][1]+allPara.ACC_Raw[gyro][2]*allPara.ACC_Raw[gyro][2]);
		ax=X_G=allPara.ACC_Raw[gyro][0]/accSum[gyro];
		ay=Y_G=allPara.ACC_Raw[gyro][1]/accSum[gyro];
		az=Z_G=allPara.ACC_Raw[gyro][2]/accSum[gyro];
		/*初始坐标为0,0,g,然后可以通过坐标变换公式轻易推导*/
		allPara.ACC_Angle[gyro][0]= safe_atan2(Y_G,Z_G);
		allPara.ACC_Angle[gyro][1]= safe_asin(-X_G);
	}
	
	//对Kp，Ki参数进行处理
	if(fabs(accSum[0]-allPara.sDta.mahony_t.accSumStatic)<0.05)
	{
     Kp=KpInit;
     Ki=KiInit;
	}else if(fabs(accSum[0]-allPara.sDta.mahony_t.accSumStatic)<0.1){
     Kp=KpInit/3;
     Ki=KiInit/3;
	}else{
     Kp=KpInit/10;
     Ki=KiInit/10;
	}
	
	vx = 2*(q[1]*q[3] - q[0]*q[2]);
  vy = 2*(q[0]*q[1] + q[2]*q[3]);
  vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
	
  ex=ay*vz-az*vy;
  ey=az*vx-ax*vz;
  ez=ax*vy-ay*vx;
	
	allPara.sDta.mahony_t.exInt = allPara.sDta.mahony_t.exInt + ex*Ki;
  allPara.sDta.mahony_t.eyInt = allPara.sDta.mahony_t.eyInt + ey*Ki;
  allPara.sDta.mahony_t.ezInt = allPara.sDta.mahony_t.ezInt + ez*Ki;
	
	
	gyroMahony[0] = w[0] + Kp*ex + allPara.sDta.mahony_t.exInt;
  gyroMahony[1] = w[1] + Kp*ey + allPara.sDta.mahony_t.eyInt;
  gyroMahony[2] = w[1] + Kp*ez + allPara.sDta.mahony_t.ezInt;
	
	QuaternionInt(allPara.sDta.quarternion,gyroMahony);
	
	Quaternion_to_Euler(allPara.sDta.quarternion,allPara.sDta.Result_Angle);
	
	allPara.sDta.Result_Angle[2]*=-1;
	
	for(int i=0;i<3;i++)
	{
		allPara.sDta.Result_Angle[i]=allPara.sDta.Result_Angle[i]/PI*180.0;
	}
}

