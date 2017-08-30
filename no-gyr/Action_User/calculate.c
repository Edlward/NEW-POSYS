#include "calculate.h"
#include "math.h"
#include "config.h"

void getJacobi(double *jacobi,const double angle);
void getSensorData(double sensorDataArray[3]);
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

double sensorData[3]={0.0,0.0,0.0};
static double gRobot[3]={0.0,0.0,0.0};

void run(void){
  
  getSensorData(sensorData);
  
  RungeKutta(gRobot);
	
}
void RungeKutta(double gRobot[3]){
  /*机器人位置，角度参数*/
  static double robot[3]={0.0,0.0,0.0};
  /*龙哥库塔的四个参数*/
  double K1[3]={0.0,0.0,0.0};
  double K2[3]={0.0,0.0,0.0};
  double K3[3]={0.0,0.0,0.0};
  double K4[3]={0.0,0.0,0.0};
  
  /*根据其阶数不同改变angle的值*/
  for(int grade=1;grade<5;grade++){
    double angleTemp=0.0;
    angleTemp=robot[2];
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
  robot[i]=robot[i]+1.0/6.0*(K1[i]+2.0*K2[i]+2.0*K3[i]+K4[i]);
  
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
  jacobi[6]=1.0/3.0/RADIUS;
  jacobi[7]=1.0/3.0/RADIUS;
  jacobi[8]=1.0/3.0/RADIUS;
};

void getSensorData(double sensorData[3]){

}
