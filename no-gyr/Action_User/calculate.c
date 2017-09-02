#include "calculate.h"
#include "math.h"
#include "config.h"
#include "usart.h"
#include "elmo.h"
#include "stm32f4xx_it.h"

void getJacobi(double *jacobi,const double angle);
void   calculateK(const double angle,double K[3]);
void RungeKutta(double gRobot[3]);


/*
result 				机器人的x，y速度，角速度
输入参数 			机器人当前角度可变,三轮速度（不可变）

x(i+1)					x(i)									
y(i+1)		    = 		y(i)		 +  1/6(	K1+2*K2+2*K3+K4 )
angle(i+1)				angle(i)

K1  =  f(robot)*PERIOD
K2  =  f(robot+1/2*K1)*PERIOD
K3  =  f(robot+1/2*K2)*PERIOD
K4  =  f(robot+K3)*PERIOD

*/

static double sensorData[3]={0.0,0.0,0.0};
static double gRobot[3]={0.0,0.0,0.0};

void run(void){
  
  RungeKutta(gRobot);

}
void RungeKutta(double gRobot[3]){
  /*龙哥库塔的四个参数*/
  double K1[3]={0.0,0.0,0.0};
  double K2[3]={0.0,0.0,0.0};
  double K3[3]={0.0,0.0,0.0};
  double K4[3]={0.0,0.0,0.0};
  
  /*根据其阶数不同改变angle的值*/
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
//		calculateK(gRobot[2],K1);
//	  for(int i=0;i<3;i++)
//    gRobot[i]=gRobot[i]+K1[i];
		
  
}

void  calculateK(const double angle,double K[3]){
  /*
  雅克比矩阵 
  与速度列向量相乘即为机器人参数斜率
  */
  double jacobi[9]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  getJacobi(jacobi,angle);
  /*机器人x，y，angle的斜率*/
  double robotRate[3]={0.0,0.0,0.0};
  
  for(int i=0;i<3;i++)
    robotRate[i]=jacobi[i*3]*sensorData[0]+jacobi[i*3+1]*sensorData[1]+jacobi[i*3+2]*sensorData[2];
  
  for(int i=0;i<3;i++)
    K[i]=robotRate[i]*PERIOD;
}

void getJacobi(double *jacobi,const double angle){
  jacobi[0]=-2.0/3.0*sin(angle);
  jacobi[1]=-sqrt(3.0)/3.0*cos(angle)+1.0/3.0*sin(angle);
  jacobi[2]=sqrt(3.0)/3.0*cos(angle)+1.0/3.0*sin(angle);
  jacobi[3]=2.0/3.0*cos(angle);
  jacobi[4]=-sqrt(3.0)/3.0*sin(angle)-1.0/3.0*cos(angle);
  jacobi[5]=sqrt(3.0)/3.0*sin(angle)-1.0/3.0*cos(angle);
  jacobi[6]=1.0/3.0/RADIUS1;
  jacobi[7]=1.0/3.0/RADIUS2;
  jacobi[8]=1.0/3.0/RADIUS3;
};

void readSensorData(void){
	
		int64_t posTem[3]={0,0,0};
		int64_t tempData[3]={0l,0l,0l};
		
		const double wheel[3]={50.0,50.0,50.0};
		
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
			
			tempData[i]=posTem[i]-posLast[i];//此处需为int64才不会发生泄漏
			
			//0到4294967296l
			if(tempData[i]>=2147483648l)
				tempData[i]-=4294967296l;
			else if(tempData[i]<=-2147483648l)
				tempData[i]+=4294967296l;
			
			if(abs_s(velLast[i])<=2&&abs_s(tempData[i])<=2){
				tempData[i]=0;
			}
				velLast[i]=tempData[i];
			
			sensorData[i]=tempData[i]/STDPULSE*2*3.141592*wheel[i]/PERIOD;
		
		}
		
		for(int i=0;i<3;i++)
		posLast[i]=posTem[i];
		
}

void debugMode(void){
		
	  float realRobot[3]={0.f,0.f,0.f};
		
		realRobot[0]=sqrt(3.0)/2*gRobot[0]+1/2*gRobot[1];
		realRobot[1]=sqrt(3.0)/2*gRobot[1]-1/2*gRobot[0];
		realRobot[2]=gRobot[2]/3.141592*180.0;
		
		USART_OUT_F(realRobot[0]);
		USART_OUT_F(realRobot[1]);
		USART_OUT_F(realRobot[2]);
		USART_OUT(USART2,(uint8_t*)"\r\n");

}


int abs_s(int i){
	if(i<0)
		return -i;
	else
		return i;
}
