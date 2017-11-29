#include "calculate.h"
#include "math.h"
#include "config.h"
#include "usart.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "arm_math.h"
#include "dma.h"
#include "spi.h"

void getJacobi(float *jacobi,const float angle,unsigned char group);
void calculateK(const float angle,float K[3],unsigned char group);
void RungeKutta(float gRobot[3],unsigned char group);
void RungeKutta2(float gRobot[3],unsigned char group);

/*
result 				�����˵�x��y�ٶȣ����ٶ�
������� 			�����˵�ǰ�Ƕȿɱ�,�����ٶȣ����ɱ䣩

x(i+1)					x(i)									
y(i+1)		    = 		y(i)		 +  1/6(	K1+2*K2+2*K3+K4 )
angle(i+1)				angle(i)

K1  =  f(robot)*PERIOD 
K2  =  f(robot+1/2*K1)*PERIOD
K3  =  f(robot+1/2*K2)*PERIOD
K4  =  f(robot+K3)*PERIOD

*/
extern Robot_t gRobot;
static float sensorData[4]={0.f,0.f,0.f,0.f};
static float gRobot_123[3]={0.f,0.f,0.f};
static float gRobot_124[3]={0.f,0.f,0.f};
static float gRobot2_123[3]={0.f,0.f,0.f};
static float gRobo2_124[3]={0.f,0.f,0.f};

void run(void){

//  RungeKutta(gRobot_123,0);
//  RungeKutta(gRobot_124,1);
  RungeKutta2(gRobot2_123,0);
  RungeKutta2(gRobo2_124,1);
	
	USART_OUT_F(sensorData[0]);
	USART_OUT_F(sensorData[1]);
	USART_OUT_F(sensorData[2]);
	USART_OUT_F(sensorData[3]);
	

	USART_OUT_F(gRobot2_123[2]/PI*180.0);
	USART_OUT_F(gRobo2_124[2]/PI*180.0);
	USART_Enter();
	

}

//�Ѿ���֤����x��y�޷����빫ʽ���Ľ׵�rungekutta�������á����Ƕ�����ȴ���ƣ�
//����x��y���ȵ�Ψһ����������С�����������ǽǶȶԼ���������С�
void RungeKutta(float gRobot[3],unsigned char group){
  /*����������ĸ�����*/
  float K1[3]={0.f,0.f,0.f};
	float K2[3]={0.0,0.0,0.0};
  float K3[3]={0.0,0.0,0.0};
  float K4[3]={0.0,0.0,0.0};
  
  /*�����������ͬ�ı�angle��ֵ*/
  for(int grade=1;grade<5;grade++){
    double angleTemp=0.0;
    angleTemp=gRobot[2];
    switch(grade){
    case 1:
      angleTemp=angleTemp;
      calculateK(angleTemp,K1,group);
      break;
    case 2:
      angleTemp+=K1[2]/2;
      calculateK(angleTemp,K2,group);
      break;
    case 3:
      angleTemp+=K2[2]/2;
      calculateK(angleTemp,K3,group);
      break;
    case 4:
      angleTemp+=K3[2];
      calculateK(angleTemp,K4,group);
      break;
    }
  }
  
  for(int i=0;i<3;i++)
		gRobot[i]=gRobot[i]+1.0f/6.0f*(K1[i]+2.0f*K2[i]+2.0f*K3[i]+K4[i]);
	
	if(gRobot[2]>PI)
		gRobot[2]-=2*PI;
	if(gRobot[2]<-PI)
		gRobot[2]+=2*PI;
}


//�Ѿ���֤����x��y�޷����빫ʽ���Ľ׵�rungekutta�������á����Ƕ�����ȴ���ƣ�
//����x��y���ȵ�Ψһ����������С�����������ǽǶȶԼ���������С�
void RungeKutta2(float gRobot[3],unsigned char group){
  /*����������ĸ�����*/
  float K1[3]={0.f,0.f,0.f};
  
	calculateK(gRobot[2],K1,group);
	for(int i=0;i<3;i++)
		gRobot[i]=gRobot[i]+K1[i];
	
	if(gRobot[2]>PI)
		gRobot[2]-=2*PI;
	if(gRobot[2]<-PI)
		gRobot[2]+=2*PI;
}



void  calculateK(const float angle,float K[3],unsigned char group){
  /*
  �ſ˱Ⱦ��� 
  ���ٶ���������˼�Ϊ�����˲���б��
  */
  float jacobi[9]={0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f};
  getJacobi(jacobi,angle,group);
  /*������x��y��angle��б��*/
  float robotRate[3]={0.f,0.f,0.f};
  
	switch(group){
		case 0:
			for(int i=0;i<3;i++)
				robotRate[i]=jacobi[i*3]*sensorData[0]+jacobi[i*3+1]*sensorData[1]+jacobi[i*3+2]*sensorData[2];
		break;
		case 1:
			for(int i=0;i<3;i++)
				robotRate[i]=jacobi[i*3]*sensorData[0]+jacobi[i*3+1]*sensorData[1]+jacobi[i*3+2]*sensorData[3];
		break;
	}
  
  for(int i=0;i<3;i++)
    K[i]=robotRate[i];
//	
//	USART_OUT_F(robotRate[0]);
//	USART_OUT_F(robotRate[1]);
//	USART_OUT_F(robotRate[2]);
//	USART_Enter();
	
	
}

#define RADIUS 					   		600.0f
#define SQRT2_2 							0.707106781186548F
void getJacobi(float *jacobi,const float angle,unsigned char group){
	
	/*v1 v2 v3�����*/
	if(group==0)
	{
		jacobi[0]=arm_sin_f32(angle)*SQRT2_2;
		jacobi[1]=(arm_cos_f32(angle) - arm_sin_f32(angle))*SQRT2_2;
		jacobi[2]=-arm_cos_f32(angle)*SQRT2_2;
		jacobi[3]=-arm_cos_f32(angle)*SQRT2_2;
		jacobi[4]=(arm_cos_f32(angle) + arm_sin_f32(angle))*SQRT2_2;
		jacobi[5]=-arm_sin_f32(angle)*SQRT2_2;
		jacobi[6]=0.5f/RADIUS;
		jacobi[7]=0.f;
		jacobi[8]=0.5f/RADIUS;
	}/*v1 v2 v4�����*/
	else if(group==1)
	{
		jacobi[0]=(arm_cos_f32(angle) + arm_sin_f32(angle))*SQRT2_2;
		jacobi[1]=-arm_sin_f32(angle)*SQRT2_2;
		jacobi[2]=-arm_cos_f32(angle)*SQRT2_2;
		jacobi[3]=-(arm_cos_f32(angle) - arm_sin_f32(angle))*SQRT2_2;
		jacobi[4]=arm_cos_f32(angle)*SQRT2_2;
		jacobi[5]=-arm_sin_f32(angle)*SQRT2_2;
		jacobi[6]=0.0f;
		jacobi[7]=0.5f/RADIUS;
		jacobi[8]=0.5f/RADIUS;
	}
};

/*��ȡ��ʱ���߹���·��*/
void readSensorData(void){
	
		int posTem[4]={0l,0l,0l,0l};
		int tempData[4]={0l,0l,0l,0l};
		
		const float wheel[4]={76.2f,76.2f,76.2f,76.2f};
		
	  static int posLast[4]={0,0,0};
		static int velLast[4]={0,0,0};
		static char initForPosLast=1;
		
		ReadActualPos(CAN1,1);
		ReadActualPos(CAN1,2);
		ReadActualPos(CAN1,3);
		ReadActualPos(CAN1,4);
		/*����ĸ����ӱ�������ֵ��������*/
	  while(!getFlagFinish());
		for(int i=0;i<4;i++){
		  posTem[i]=GetPos()[i];
			/*���³�ʼλ�õ�ֵ����Ϊ��һ�μ���ʱû����һ�ε�ֵ*/
			if(initForPosLast){
				posLast[i]=posTem[i];
			}
		}
				initForPosLast=0;
		
		for(int i=0;i<4;i++){
			
			tempData[i]=posTem[i]-posLast[i];//�˴���Ϊint64�Ų��ᷢ��й©
			
			if(tempData[i]>2048)
				tempData[i]-=4096;
			else if(tempData[i]<-2048)
				tempData[i]+=4096;
			
			if(abs_s(velLast[i])<=2&&abs_s(tempData[i])<=2){
				tempData[i]=0;
			}
			
			velLast[i]=tempData[i];
			
			sensorData[i]=tempData[i]/STDPULSE*2.f*3.141592f*wheel[i];
			
			posLast[i]=posTem[i];
		}
	
}


int abs_s(int i){
	if(i<0)
		return -i;
	else
		return i;
}
