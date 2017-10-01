#include "calculate.h"
#include "math.h"
#include "config.h"
#include "usart.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "arm_math.h"
#include "dma.h"
#include "spi.h"

void getJacobi(float *jacobi,const float angle);
void   calculateK(const float angle,float K[3]);
void RungeKutta(float gRobot[3]);
void RungeKutta2(float gRobot[3]);

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

//已经验证由于x，y无法代入公式，四阶的rungekutta并不好用。（角度性能却相似）
//增加x，y精度的唯一方法就是缩小计算间隔。但是角度对间隔并不敏感。
void RungeKutta(float gRobot[3]){
  /*龙哥库塔的四个参数*/
  float K1[3]={0.f,0.f,0.f};
	float K2[3]={0.0,0.0,0.0};
  float K3[3]={0.0,0.0,0.0};
  float K4[3]={0.0,0.0,0.0};
  
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
  gRobot[i]=gRobot[i]+1.0f/6.0f*(K1[i]+2.0f*K2[i]+2.0f*K3[i]+K4[i]);

}


//已经验证由于x，y无法代入公式，四阶的rungekutta并不好用。（角度性能却相似）
//增加x，y精度的唯一方法就是缩小计算间隔。但是角度对间隔并不敏感。
void RungeKutta2(float gRobot[3]){
  /*龙哥库塔的四个参数*/
  float K1[3]={0.f,0.f,0.f};
  
	calculateK(gRobot[2],K1);
	for(int i=0;i<3;i++)
  gRobot[i]=gRobot[i]+K1[i];
}



void  calculateK(const float angle,float K[3]){
  /*
  雅克比矩阵 
  与速度列向量相乘即为机器人参数斜率
  */
  float jacobi[9]={0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f};
  getJacobi(jacobi,angle);
  /*机器人x，y，angle的斜率*/
  float robotRate[3]={0.f,0.f,0.f};
  
  for(int i=0;i<3;i++)
    robotRate[i]=jacobi[i*3]*sensorData[0]+jacobi[i*3+1]*sensorData[1]+jacobi[i*3+2]*sensorData[2];
  
  for(int i=0;i<3;i++)
    K[i]=robotRate[i]*PERIOD;
}

void getJacobi(float *jacobi,const float a){
	const float a1=2.125811f,a2=2.097885f;
	const float d1=35.69f,d2=35.69f,d3=35.69f;
//  jacobi[0]=(d3*arm_cos_f32(a + a1) - d2*arm_cos_f32(a - a2))/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[1]=-(d3*arm_cos_f32(a) - d1*arm_cos_f32(a - a2))/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[2]=-(d1*arm_cos_f32(a + a1) - d2*arm_cos_f32(a))/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[3]=(d3*arm_sin_f32(a + a1) - d2*arm_sin_f32(a - a2))/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[4]=-(d3*arm_sin_f32(a) - d1*arm_sin_f32(a - a2))/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[5]=-(d1*arm_sin_f32(a + a1) - d2*arm_sin_f32(a))/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[6]=-arm_sin_f32(a1 + a2)/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[7]=arm_sin_f32(a2)/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
//  jacobi[8]=arm_sin_f32(a1)/(d2*arm_sin_f32(a2) - d1*arm_sin_f32(a1 + a2) + d3*arm_sin_f32(a1));
  jacobi[0]=(d3*arm_cos_f32(a + a1) - d2*arm_cos_f32(a - a2))/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[1]=-(d3*arm_cos_f32(a) - d1*arm_cos_f32(a - a2))/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[2]=-(d1*arm_cos_f32(a + a1) - d2*arm_cos_f32(a))/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[3]=(d3*arm_sin_f32(a + a1) - d2*arm_sin_f32(a - a2))/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[4]=-(d3*arm_sin_f32(a) - d1*arm_sin_f32(a - a2))/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[5]=-(d1*arm_sin_f32(a + a1) - d2*arm_sin_f32(a))/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[6]=0.882947f/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[7]=0.864275f/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
  jacobi[8]=0.849893f/(30.845981f - d1*arm_sin_f32(a1 + a2) + 30.332671f );
};

void readSensorData(void){
	static uint16_t data_last[3]={0,0};
	static int totalDistance[3]={0,0,0};
	static int iForStartErrorOfVell=0;
	
	uint16_t data[3];
	int16_t vell[3];
	
	const float wheel[3]={25.395f,25.34f,25.4f};
	
	data[0]=SPI_ReadAS5045(0);
	data[1]=SPI_ReadAS5045(1);
	data[2]=SPI_ReadAS5045(2);
	
	vell[0]= (data[0]-data_last[0]);
	vell[1]= (data[1]-data_last[1]);
	vell[2]= (data[2]-data_last[2]);
		
	data_last[0]=data[0];
	data_last[1]=data[1];
	data_last[2]=data[2];
	
	/*看这个编码器的范围*/
	if(vell[0]>2048)
		vell[0]-=4096;
	if(vell[0]<-2048)
		vell[0]+=4096;
	
	if(vell[1]>2048)
		vell[1]-=4096;
	if(vell[1]<-2048)
		vell[1]+=4096;
	
	if(vell[2]>2048)
		vell[2]-=4096;
	if(vell[2]<-2048)
		vell[2]+=4096;

	for(int i=0;i<3;i++)
		totalDistance[i]+=vell[i];
	

	#ifdef CORRECT	
			/*清0*/
			if(getI()==1){
				setI(0);
				for(int i=0;i<3;i++)
					totalDistance[i]=0;
				USART_OUT_CHAR("finish\r\n");
			/*发数*/
			}else if(getI()==2){
				setI(0);
				for(int i=0;i<3;i++)
					USART_OUTF(totalDistance[i]);
				USART_OUT_CHAR("\r\n");
			}
			
//			USART_OUTF(vell[0]);
//			USART_OUTF(vell[1]);
//			USART_OUTF(vell[2]);
//			USART_OUTF(totalDistance[0]);
//			USART_OUTF(totalDistance[1]);
//			USART_OUTF(totalDistance[2]);
//			USART_OUT_CHAR("\r\n");
			
				
	#endif
	
	iForStartErrorOfVell++;
	if(iForStartErrorOfVell>100){
		iForStartErrorOfVell=101;
		for(int i=0;i<3;i++)
		sensorData[i]=vell[i]/4096.f*2.f*3.141592f*wheel[i]/PERIOD;
	}

}

void debugMode(void){	
		
//	  static float realRobot[3]={0.f,0.f,0.f};
		
//		realRobot[0]=sqrt(3.f)/2.f*gRobot[0]+1.f/2.f*gRobot[1];
//		realRobot[1]=sqrt(3.f)/2.f*gRobot[1]-1.f/2.f*gRobot[0];
//		realRobot[2]=gRobot[2]/3.141592*180.f;
		#ifndef CORRECT
			static int i=0;  
		  switch(i)
			{
				case 0:
					USART_OUT_F(gRobot[0]);
					i++;
					break;
				case 1:
					USART_OUT_F(gRobot[1]);
					i++;
					break;
				case 2:
					USART_OUT_F(gRobot[2]/3.141592*180.f);
					i++;
					break;
				case 3:
					USART_OUT_F(gRobot2[0]);
					i++;
					break;
				case 4:
					USART_OUT_F(gRobot2[1]);
					i++;
					break;
				case 5:
					USART_OUT_F(gRobot2[2]/3.141592*180.f);
					i++;
					break;
				case 6:
					USART_OUT(USART1,"\r\n");
					i=0;
					break;
				case 7:
					break;
				case 8:
					break;
			
			}
			//USART_OUT_CHAR("\r\n");
		#else
			
			
		#endif

}


int abs_s(int i){
	if(i<0)
		return -i;
	else
		return i;
}
