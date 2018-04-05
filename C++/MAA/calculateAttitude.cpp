
/**
  ******************************************************************************
  * @file    calculateAttitude.cpp
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2017.3.22
  * @brief   用于姿态计算
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "calculateAttitude.h"
#include "timer.h"
#include "ADXRS453.h"
#include "I3G4250D.h"
#include "LSM303AGR.h"
#include "usart.h"
#include "action_matrix.h"
#include "attitudeAngle.h"
#include "adaptiveUnscentedKalmanFilter.h"
#include "user.h"
#include "signalProcess.h"
#include "pos.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static const uint32_t calculateTime=10*100;

static threeAxis eulerAngle;

static threeAxis dataI3G4250D={0,0,0};
static float tempI3G4250D=0.0f;

static threeAxis dataLSM303AGR_acc={0,0,0};
static threeAxis dataLSM303AGR_mag={0,0,0};

static oneAxis 	 dataADXRS453=0.0f;	

static threeAxis magSphere={111.6486f,-43.8525f,117.3009f};


static action_matrix initMatrix(3,1);
static action_matrix initQ4(4,1,MATRIX_ZERO);


static action_matrix UKFpre(const action_matrix& X,const action_matrix& Y);
static action_matrix UKFmeasure(const action_matrix& X);

class aukf9axis:public AUKF
{
private:
	action_matrix* rMag;
	action_matrix* rAcc;

public:
	aukf9axis()
		:AUKF(1,0,0.1,action_matrix(4,1,MATRIX_ZERO),action_matrix(4,4,MATRIX_I),action_matrix(4,4,MATRIX_I),action_matrix(3,3,MATRIX_I),action_matrix(4,4,MATRIX_I),UKFpre,UKFmeasure,10)
	{
		rMag=new action_matrix[rLen];
		rAcc=r;
	}
	virtual ~aukf9axis()
	{
		delete[] rMag;
	}
	action_matrix fuse(const action_matrix& gyro,const	action_matrix& acc,const	action_matrix& mag)
	{
		predictX(gyro);
		
		R=action_matrix(3,3,MATRIX_I)*1;
		
		//融合加速度计
		r=rAcc;									//将存储空间指向加速度计的存储
		initMatrix[0][0]=0;			//认为0位置时候的加速度计值为【0,0,1】
		initMatrix[1][0]=0;
		initMatrix[2][0]=1;
		predictZ();							//通过陀螺仪积分得到的姿态来预估会测量得到的加速度计值
		adaptiveR(acc);					//自适应的计算R系数
		estimateX(acc);					//通过前面得到的矩阵融合得到姿态
		
		R=action_matrix(3,3,MATRIX_I)*1;
		
		if(MAG_IS_ENABLE)
		{
			r=rMag;									//同上
			initMatrix[0][0]=dataLSM303AGR_mag.x;
			initMatrix[1][0]=dataLSM303AGR_mag.y;
			initMatrix[2][0]=dataLSM303AGR_mag.z;
			
			predictZ();
			adaptiveR(mag);
			estimateX(mag);
		}
		
		r=rAcc;									//为防止析构的时候一部分数据不被释放，正常情况这个类创建的对象应该不会被释放
		
		//四元数归一化
		float sum=0;
		for(uint8_t i=0;i<4;i++)
		{
			sum+=X[i][0]*X[i][0];
		}
		sum=sqrt(sum);
		for(uint8_t i=0;i<4;i++)
		{
			X[i][0]=X[i][0]/sum;
		}
		
		return X;
	}
};
static aukf9axis aukf;
//static AUKF aukf(0.2,2,action_matrix(4,1,MATRIX_ZERO),action_matrix(4,4,MATRIX_I),action_matrix(4,4,MATRIX_I),action_matrix(3,3,MATRIX_I),action_matrix(4,4,MATRIX_I),UKFpre,UKFmeasure,10);

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
static action_matrix UKFpre(const action_matrix& X,const action_matrix& Y)
{
	auto pre=integral(X,Y/180.0f*3.1415926f,0.01f);
	return pre;
}
static action_matrix UKFmeasure(const action_matrix& X)
{
	action_matrix trans;
	trans=getTransMatrix(X);
	
	action_matrix temp;
	temp=initMatrix;
	
	temp=(!trans)*temp;
	return temp;
}
static void magAccFixZeroRate(void)
{
	static threeAxis accData[calculateTime];
	static threeAxis magData[calculateTime];
	static float     gyroData[calculateTime];
	static threeAxis gyroLowData[calculateTime];
	static uint32_t countAvoidErr=0;
	
	countAvoidErr++;
	shiftRightData(accData,calculateTime);
	shiftRightData(magData,calculateTime);
	shiftRightData(gyroData,calculateTime);
	shiftRightData(gyroLowData,calculateTime);
	
	accData[0]=getLSM303AGR_Acc().getData();
	magData[0]=getLSM303AGR_Mag().getData();
	gyroData[0]=getADXRS453().getData();
	gyroLowData[0]=getI3G4250D().getData();
	
	threeAxis stdAcc[calculateTime/100];
	threeAxis stdMag[calculateTime/100];
	uint16_t i=0;
	for(i=0;i<calculateTime/100;i++)
	{
		stdAcc[i]=stdData(accData+i*100,100);
		stdMag[i]=stdData(magData+i*100,100);
		if(stdAcc[i].x>5||stdAcc[i].y>5||stdAcc[i].z>7||
			 stdMag[i].x>5||stdMag[i].y>5||stdMag[i].z>5)
		{
			break;
		}
	}
	if((i==calculateTime/100)&&(countAvoidErr>=calculateTime))
	{
		float mean=meanData(gyroData+calculateTime/4,calculateTime/2);
		threeAxis meanLow=meanData(gyroLowData+calculateTime/4,calculateTime/2);
		
		dataADXRS453+=(mean-dataADXRS453)/(calculateTime/5);
		dataI3G4250D.x+=(meanLow.x-dataI3G4250D.x)/(calculateTime/5);
		dataI3G4250D.y+=(meanLow.y-dataI3G4250D.y)/(calculateTime/5);
	}
}
static threeAxis Quaternion_to_Euler(const action_matrix& quaternion)
{
	threeAxis Rad;
	float q0, q1, q2, q3;
	float sum;

	q0 = quaternion[0][0];
	q1 = quaternion[1][0];
	q2 = quaternion[2][0];
	q3 = quaternion[3][0];

	sum = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / sum;
	q1 = q1 / sum;
	q2 = q2 / sum;
	q3 = q3 / sum;

	Rad.x = asin(2.0f*(q2*q3 + q0*q1));
	Rad.y = atan2(-2 * q1 * q3 + 2 * q0 * q2, q3*q3 - q2 * q2 - q1 * q1 + q0 * q0);
	Rad.z = atan2(2 * q1*q2 - 2 * q0*q3, q2*q2 - q3*q3 + q0*q0 - q1*q1);
	return Rad;
}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void initAHRS(void)
{
	threeAxis newDataI3G4250D[calculateTime/100];
	float     newTempI3G4250D[calculateTime/100];
	float			newDataADXRS453[calculateTime/100];
	
	threeAxis newDataLSM303AGR_acc[calculateTime/100];
	threeAxis newDataLSM303AGR_mag[calculateTime/100];
	
	dataI3G4250D=0;
	dataLSM303AGR_acc=0;
	dataADXRS453=0;
	dataLSM303AGR_mag=0;
	
	uint16_t count=0;
	uint16_t countSec=0;
	
	//下面求初始化的均值
	while((!DEVICE_IS_RUNNING)||(countSec<=(calculateTime/100)))
	{
		while(!getTimeFlag());
		getLSM303AGR_Acc().updateData();
		getLSM303AGR_Mag().updateData();
		getI3G4250D().updateData();
		UpdateEncoder();
		auto mag=getLSM303AGR_Mag().getData();
		encoderMagFix(mag);
		mag=mag-magSphere;
		
		dataI3G4250D=dataI3G4250D+getI3G4250D().getData();
		tempI3G4250D=tempI3G4250D+getI3G4250D().temp;
		dataLSM303AGR_acc=dataLSM303AGR_acc+getLSM303AGR_Acc().getData();
		dataADXRS453=dataADXRS453+getADXRS453().getData();
		dataLSM303AGR_mag=dataLSM303AGR_mag+mag;
		count++;
		if(count%100==0)
		{
			countSec++;
			shiftRightData(newDataI3G4250D,calculateTime/100);
			shiftRightData(newTempI3G4250D,calculateTime/100);
			shiftRightData(newDataADXRS453,calculateTime/100);
			
			shiftRightData(newDataLSM303AGR_acc,calculateTime/100);
			shiftRightData(newDataLSM303AGR_mag,calculateTime/100);
			
			newDataI3G4250D[0]=dataI3G4250D/100;
			newTempI3G4250D[0]=tempI3G4250D/100;
			newDataADXRS453[0]=dataADXRS453/100;
			
			newDataLSM303AGR_acc[0]=dataLSM303AGR_acc/100;
			newDataLSM303AGR_mag[0]=dataLSM303AGR_mag/100;
			
			dataADXRS453=0;
			dataI3G4250D=0;
			tempI3G4250D=0;
			dataLSM303AGR_acc=0;
			dataLSM303AGR_mag=0;
		}
	}
	
	dataI3G4250D=meanData(newDataI3G4250D,calculateTime/100);
	tempI3G4250D=meanData(newTempI3G4250D,calculateTime/100);
	
	dataLSM303AGR_acc=meanData(newDataLSM303AGR_acc,calculateTime/100);
	dataLSM303AGR_mag=meanData(newDataLSM303AGR_mag,calculateTime/100);
	
	dataADXRS453=meanData(newDataADXRS453,calculateTime/100);
	
	//归一化得到方向余弦
	dataLSM303AGR_acc=dataLSM303AGR_acc/abs(dataLSM303AGR_acc);
	dataLSM303AGR_mag=dataLSM303AGR_mag/abs(dataLSM303AGR_mag);
	
	//通过方向余弦计算欧垃角，这里我们默认初始的时候航向角为0
	double angleX,angleY;
	angleX=asin(dataLSM303AGR_acc.y);
	angleY=atan2(-dataLSM303AGR_acc.x,dataLSM303AGR_acc.z);
	
	aukf.R=aukf.R*10;
	aukf.Q=aukf.Q*0.01;
	aukf.P=aukf.P*0.0001;
	
	aukf.X[0][0]=sqrt(cos(angleY)+cos(angleX)+cos(angleY)*cos(angleX)+1)/2.0;
	aukf.X[1][0]=(sin(angleX)+sin(angleX)*cos(angleY))/4.0/aukf.X[0][0];
	aukf.X[2][0]=(sin(angleY)+sin(angleY)*cos(angleX))/4.0/aukf.X[0][0];
	aukf.X[3][0]=sin(angleX)*sin(angleY)/4.0/aukf.X[0][0];

	initQ4=aukf.X;

	action_matrix magVector(3,1,MATRIX_ZERO);
	magVector[0][0]=dataLSM303AGR_mag.x;
	magVector[1][0]=dataLSM303AGR_mag.y;
	magVector[2][0]=dataLSM303AGR_mag.z;
	
	magVector=getTransMatrix(aukf.X)*magVector;
	
	action_matrix temp=getTransMatrix(aukf.X);
	
	dataLSM303AGR_mag.x=magVector[0][0];
	dataLSM303AGR_mag.y=magVector[1][0];
	dataLSM303AGR_mag.z=magVector[2][0];
	
}
void updateAHRS(void)
{
	action_matrix zAcc(3,1);
	action_matrix zMag(3,1);
	action_matrix w(3,1,MATRIX_ZERO);
	
	magAccFixZeroRate();
	
	auto acc=getLSM303AGR_Acc().getData();
	auto gyro=getI3G4250D().getData();
	auto mag=getLSM303AGR_Mag().getData();
	auto gyroHigh=getADXRS453().getData();
	
	encoderMagFix(mag);
	cout<<mag.x<<'\t'<<mag.y<<'\t'<<mag.z<<'\t';
	
	mag=mag-magSphere;
	
	acc=acc/abs(acc);
	zAcc[0][0]=acc.x;
	zAcc[1][0]=acc.y;
	zAcc[2][0]=acc.z;

	mag=mag/abs(mag);
	zMag[0][0]=mag.x;
	zMag[1][0]=mag.y;
	zMag[2][0]=mag.z;
	
	w[0][0]=gyro.x-dataI3G4250D.x;
	w[1][0]=gyro.y-dataI3G4250D.y;
	
	if(abs(gyro.z)<250)
	{
		w[2][0]=gyroHigh-dataADXRS453;
	}
	else
	{
		w[2][0]=gyro.z-dataI3G4250D.z-(getI3G4250D().temp-tempI3G4250D)*0.073f;
	}
	
	aukf.fuse(w,zAcc,zMag);
	
	eulerAngle=Quaternion_to_Euler(aukf.X)/3.1415926*180.0;
	
	cout<<dataADXRS453<<'\t'<</*gyro.z<<'\t'<<*/eulerAngle.z<<'\t'<<eulerAngle.x<<'\t'<<eulerAngle.y<<'\t'/*<<dataADXRS453<<'\t'*/;
}
threeAxis getEulerAngle(void)
{
	return eulerAngle;
}
void resetAttitude(void)
{
	aukf.X=initQ4;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
