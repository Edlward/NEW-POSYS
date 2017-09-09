#include "calculate.h"
#include "math.h"
#include "config.h"
#include "usart.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "arm_math.h"
//#include "dma.h"

void getJacobi(float *jacobi,const float angle);
void   calculateK(const float angle,float K[3]);
void RungeKutta(float gRobot[3]);
void RungeKutta2(float gRobot[3]);

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

static float sensorData[3]={0.f,0.f,0.f};
static float gRobot[3]={0.f,0.f,0.f};
static float gRobot2[3]={0.f,0.f,0.f};

void run(void){
	
	#ifdef SIMULATION
	static uint32_t time=0;
	if(time<1.f/PERIOD*180.f)
		time++;
	else
		time=0;
	
	
  sensorData[0]=time*PERIOD;
  sensorData[1]=time*PERIOD*time*PERIOD*5;
  sensorData[2]=time*PERIOD*time*PERIOD*3;
	
	if(time==0){
  sensorData[0]=0;
  sensorData[1]=0;
  sensorData[2]=0;
	time=1000000;
	}
	#endif
		
  RungeKutta(gRobot);
  RungeKutta2(gRobot2);

}

//�Ѿ���֤����x��y�޷����빫ʽ���Ľ׵�rungekutta�������á����Ƕ�����ȴ���ƣ�
//����x��y���ȵ�Ψһ����������С�����������ǽǶȶԼ���������С�
void RungeKutta(float gRobot[3]){
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
      calculateK(angleTemp,K1);
      break;
    case 2:
      angleTemp+=K1[2]/2;
      calculateK(angleTemp,K2);
      break;
    case 3:
      angleTemp+=K2[2]/2;
      calculateK(angleTemp,K3);
      break;
    case 4:
      angleTemp+=K3[2];
      calculateK(angleTemp,K4);
      break;
    }
  }
  
  for(int i=0;i<3;i++)
  gRobot[i]=gRobot[i]+1.0/6.0*(K1[i]+2.0*K2[i]+2.0*K3[i]+K4[i]);

}


//�Ѿ���֤����x��y�޷����빫ʽ���Ľ׵�rungekutta�������á����Ƕ�����ȴ���ƣ�
//����x��y���ȵ�Ψһ����������С�����������ǽǶȶԼ���������С�
void RungeKutta2(float gRobot[3]){
  /*����������ĸ�����*/
  float K1[3]={0.f,0.f,0.f};
  
	calculateK(gRobot[2],K1);
	for(int i=0;i<3;i++)
  gRobot[i]=gRobot[i]+K1[i];
}



void  calculateK(const float angle,float K[3]){
  /*
  �ſ˱Ⱦ��� 
  ���ٶ���������˼�Ϊ�����˲���б��
  */
  float jacobi[9]={0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f};
  getJacobi(jacobi,angle);
  /*������x��y��angle��б��*/
  float robotRate[3]={0.f,0.f,0.f};
  
  for(int i=0;i<3;i++)
    robotRate[i]=jacobi[i*3]*sensorData[0]+jacobi[i*3+1]*sensorData[1]+jacobi[i*3+2]*sensorData[2];
  
  for(int i=0;i<3;i++)
    K[i]=robotRate[i]*PERIOD;
}

void getJacobi(float *jacobi,const float angle){
  jacobi[0]=-2.f/3.f*arm_sin_f32(angle);
  jacobi[1]=-sqrt(3.f)/3.f*arm_cos_f32(angle)+1.f/3.f*arm_sin_f32(angle);
  jacobi[2]=sqrt(3.f)/3.f*arm_cos_f32(angle)+1.f/3.f*arm_sin_f32(angle);
  jacobi[3]=2.f/3.f*arm_cos_f32(angle);
  jacobi[4]=-sqrt(3.f)/3.f*arm_sin_f32(angle)-1.f/3.f*arm_cos_f32(angle);
  jacobi[5]=sqrt(3.f)/3.f*arm_sin_f32(angle)-1.f/3.f*arm_cos_f32(angle);
  jacobi[6]=1.f/3.f/RADIUS1;
  jacobi[7]=1.f/3.f/RADIUS2;
  jacobi[8]=1.f/3.f/RADIUS3;
};

void readSensorData(void){
	
		int64_t posTem[3]={0l,0l,0l};
		int64_t tempData[3]={0l,0l,0l};
		
		const float wheel[3]={50.f,50.f,50.f};
		
	  static int64_t posLast[3]={0,0,0};
		static int velLast[3]={0,0,0};
		static char initForPosLast=1;
		
		ReadActualPos(1);
		ReadActualPos(2);
		ReadActualPos(3);
	  while(!getFlagFinish());
		for(int i=0;i<3;i++){
		  posTem[i]=GetPos()[i];
			if(initForPosLast){
				posLast[i]=posTem[i];
			}
		}
		initForPosLast=0;
		
		for(int i=0;i<3;i++){
			
			tempData[i]=posTem[i]-posLast[i];//�˴���Ϊint64�Ų��ᷢ��й©
			
			//0��4294967296l
			if(tempData[i]>=2147483648l)
				tempData[i]-=4294967296l;
			else if(tempData[i]<=-2147483648l)
				tempData[i]+=4294967296l;
			
			if(abs_s(velLast[i])<=2&&abs_s(tempData[i])<=2){
				tempData[i]=0;
			}
				velLast[i]=tempData[i];
			
			sensorData[i]=tempData[i]/STDPULSE*2.f*3.141592f*wheel[i]/PERIOD;
		
		}
		
		for(int i=0;i<3;i++)
		posLast[i]=posTem[i];
		
}

void debugMode(void){	
		
	  static float realRobot[3]={0.f,0.f,0.f};
		
//		realRobot[0]=sqrt(3.f)/2.f*gRobot[0]+1.f/2.f*gRobot[1];
//		realRobot[1]=sqrt(3.f)/2.f*gRobot[1]-1.f/2.f*gRobot[0];
//		realRobot[2]=gRobot[2]/3.141592*180.f;
//	
  	USART_OUT_F(gRobot[0]);
  	USART_OUT_F(gRobot[1]);
   	USART_OUT_F(gRobot[2]);
  	USART_OUT_F(gRobot2[0]);
  	USART_OUT_F(gRobot2[1]);
   	USART_OUT_F(gRobot2[2]);
		USART_OUT(USART2,(uint8_t*)"\r\n");
  	//USART_OUT_CHAR("\r\n");
		

}


int abs_s(int i){
	if(i<0)
		return -i;
	else
		return i;
}
