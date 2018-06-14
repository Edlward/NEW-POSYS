#include "mahony.h"
#include "config.h"
#include "self_math.h"
#include "quarternion.h"

extern AllPara_t allPara;

//mahony�˲���Kpϵ����ʼֵ
float KpInit=2.f;
//mahony�˲���Kiϵ����ʼֵ
float KiInit=0.005f;

void InitQuarternion(void)
{
	//quaternionMahony=Euler_to_Quaternion([atan2d(ay(1),az(1)) asind(-ax(1)) 0]);
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		float X_G,Y_G,Z_G;
		float tempSum=sqrt(allPara.ACC_Raw[gyro][0]*allPara.ACC_Raw[gyro][0]+allPara.ACC_Raw[gyro][1]*allPara.ACC_Raw[gyro][1]+allPara.ACC_Raw[gyro][2]*allPara.ACC_Raw[gyro][2]);
		X_G=allPara.ACC_Raw[gyro][0]/tempSum;
		Y_G=allPara.ACC_Raw[gyro][1]/tempSum;
		Z_G=allPara.ACC_Raw[gyro][2]/tempSum;
		/*��ʼ����Ϊ0,0,g,Ȼ�����ͨ������任��ʽ�����Ƶ�*/
		allPara.ACC_Angle[gyro][0]= safe_atan2(Y_G,Z_G);
		allPara.ACC_Angle[gyro][1]= safe_asin(-X_G);
	}
	
	allPara.sDta.Result_Angle[0]=allPara.ACC_Angle[0][0];
	allPara.sDta.Result_Angle[1]=allPara.ACC_Angle[0][1];
	allPara.sDta.Result_Angle[2]=0.0;
	Euler_to_Quaternion(allPara.sDta.Result_Angle,allPara.sDta.quarternion);
	//��ʼ�����ٶȿ�ʼ��ʱ����ܺ�
	allPara.sDta.mahony_t.accSumStatic=sqrt(allPara.ACC_Raw[0][0]*allPara.ACC_Raw[0][0]+allPara.ACC_Raw[0][1]*allPara.ACC_Raw[0][1]+allPara.ACC_Raw[0][2]*allPara.ACC_Raw[0][2]);
}



void MahonyFilter(void)
{
	//���õ�
	long double Kp=0.0;
	long double Ki=0.0;
	long double ex,ey,ez=0.0;
	long double vx,vy,vz=0.0;
	long double ax,ay,az=0.0;
	long double w[3]={0.0};
	long double gyroMahony[3]={0.0};
	long double accSum[GYRO_NUMBER]={0.f};
	long double q[4]={0.0};
	
	for(int i=0;i<4;i++)
		q[i]=allPara.sDta.quarternion[i];

	//�Խ��ٶȽ��д���
	for(int i=0;i<3;i++)
	{
		w[i]=allPara.GYRO_Real[i];
//		if(i!=2)
//		{
//			if(fabs(w[i])<0.1f)//��λ ��/s
//				w[i]=0.f;
//		}
//		else
//		{
//			#ifdef AUTOCAR
//			if((allPara.sDta.flag&STATIC_FORCE))//��λ ��/s
//				w[i]=0.f;
//			#else
//			if((allPara.sDta.flag&STATIC_FORCE)||(fabs(FilterVell(allPara.vell[1]))<=20.f)))
//				w[i]=0.f;
//			#endif
//		}
	}
	
	//�Լ��ٶȽ��д���
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		double X_G,Y_G,Z_G;
		accSum[gyro]=sqrt(allPara.ACC_Raw[gyro][0]*allPara.ACC_Raw[gyro][0]+allPara.ACC_Raw[gyro][1]*allPara.ACC_Raw[gyro][1]+allPara.ACC_Raw[gyro][2]*allPara.ACC_Raw[gyro][2]);
		ax=X_G=allPara.ACC_Raw[gyro][0]/accSum[gyro];
		ay=Y_G=allPara.ACC_Raw[gyro][1]/accSum[gyro];
		az=Z_G=allPara.ACC_Raw[gyro][2]/accSum[gyro];
		/*��ʼ����Ϊ0,0,g,Ȼ�����ͨ������任��ʽ�����Ƶ�*/
		allPara.ACC_Angle[gyro][0]= safe_atan2(Y_G,Z_G);
		allPara.ACC_Angle[gyro][1]= safe_asin(-X_G);
	}
	
	//��Kp��Ki�������д���
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
	
	vx = 2.0*(q[1]*q[3] - q[0]*q[2]);
  vy = 2.0*(q[0]*q[1] + q[2]*q[3]);
  vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
	
//	for(int i=0;i<4;i++)
//		USART_OUTByDMAF(q[i]);
//	USART_OUTByDMAF(vx);
//	USART_OUTByDMAF(vy);
//	USART_OUTByDMAF(vz);
  ex=ay*vz-az*vy;
  ey=az*vx-ax*vz;
  ez=ax*vy-ay*vx;
	
//	USART_OUTByDMAF(Kp*ex);
//	USART_OUTByDMAF(Kp*ey);
//	USART_OUTByDMAF(Kp*ez);
	
	allPara.sDta.mahony_t.exInt = allPara.sDta.mahony_t.exInt + ex*Ki;
  allPara.sDta.mahony_t.eyInt = allPara.sDta.mahony_t.eyInt + ey*Ki;
  allPara.sDta.mahony_t.ezInt = allPara.sDta.mahony_t.ezInt + ez*Ki;
	
//	USART_OUTByDMAF(allPara.sDta.mahony_t.exInt);
//	USART_OUTByDMAF(allPara.sDta.mahony_t.eyInt);
//	USART_OUTByDMAF(allPara.sDta.mahony_t.ezInt);
	
	gyroMahony[0] = w[0] + Kp*ex + allPara.sDta.mahony_t.exInt;
  gyroMahony[1] = w[1] + Kp*ey + allPara.sDta.mahony_t.eyInt;
  gyroMahony[2] = w[2] + Kp*ez + allPara.sDta.mahony_t.ezInt;
	
//	#ifdef AUTOCAR
//		if((allPara.sDta.flag&STATIC_FORCE))//��λ ��/s
//			for(int i=0;i<3;i++)
//				gyroMahony[i]=0.0;
//	#else
//		if((allPara.sDta.flag&STATIC_FORCE)||(fabs(FilterVell(allPara.vell[1]))<=20.f)))
//			for(int i=0;i<3;i++)
//				gyroMahony[i]=0.0;
//	#endif
		
	QuaternionInt(allPara.sDta.quarternion,gyroMahony);
	
	Quaternion_to_Euler(allPara.sDta.quarternion,allPara.sDta.Result_Angle);
	
	allPara.sDta.Result_Angle[2]*=-1;
	
	for(int i=0;i<3;i++)
	{
		allPara.sDta.Result_Angle[i]=allPara.sDta.Result_Angle[i]/PI*180.0;
	}
//	USART_OUTByDMAF(gyroMahony[0]);
//	USART_OUTByDMAF(gyroMahony[1]);
//	USART_OUTByDMAF(gyroMahony[2]);
//	USART_OUTByDMAF(allPara.sDta.Result_Angle[0]);
//	USART_OUTByDMAF(allPara.sDta.Result_Angle[1]);
//	USART_OUTByDMAF(allPara.sDta.Result_Angle[2]);
//	USART_EnterByDMA();
}

