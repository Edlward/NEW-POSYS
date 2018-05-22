#include "arm_math.h"
#include "figureAngle.h"

extern AllPara_t allPara;

/*这是惯性导航上的算法，人家是  z  x  y 旋转的，所以x是-90到90*/
/**
* @brief  将四元数转换为欧拉角
* @param  quaternion: 需要转换的四元数
* @retval 四元数对应的欧拉角
*/
void Quaternion_to_Euler(const double quaternion[4],double Rad[3] )
{
  double q0,q1,q2,q3;
  double sum;
  
  q0=quaternion[0];
  q1=quaternion[1];
  q2=quaternion[2];
  q3=quaternion[3];
  
  sum=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0=q0/sum;
  q1=q1/sum;
  q2=q2/sum;
  q3=q3/sum;
  
  Rad[0]= atan2(2 * q0 * q1 + 2 * q2 * q3, q3*q3 - q2 * q2 - q1 * q1 +q0 * q0);
  Rad[1]= safe_asin(2.0*(-q1*q3 + q0*q2));
  Rad[2]= atan2(2*q1*q2+2*q0*q3,-q2*q2-q3*q3+q0*q0+q1*q1);
}

/**
* @brief  将欧拉角转换为四元数
* @param  quaternion: 需要转换的欧拉角
* @retval 四元数对应的四元数
*/
void Euler_to_Quaternion(const double Rad[3],double quaternion[4])
{
	quaternion[0]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[1]=arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)-arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[2]=arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[3]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2)-arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2);
}

void getJacobi(double dif_quarterion[4],const double quaternion[4],const double data[3]){
  dif_quarterion[0]=(-quaternion[1]*data[0] - quaternion[2]*data[1] - quaternion[3]*data[2])*0.5;
  dif_quarterion[1]=( quaternion[0]*data[0] + quaternion[2]*data[2] - quaternion[3]*data[1])*0.5;
  dif_quarterion[2]=( quaternion[0]*data[1] - quaternion[1]*data[2] + quaternion[3]*data[0])*0.5;
  dif_quarterion[3]=( quaternion[0]*data[2] + quaternion[1]*data[1] - quaternion[2]*data[0])*0.5;
};
void  calculateK(double quaternion[4],double dif_quarterion[4],double data[3]){
  
  double jacobi[4];
  getJacobi(jacobi,quaternion,data);
  
  dif_quarterion[0]=jacobi[0]*dT;
  dif_quarterion[1]=jacobi[1]*dT;
  dif_quarterion[2]=jacobi[2]*dT;
  dif_quarterion[3]=jacobi[3]*dT; 
}

/**
* @brief  针对四元数与角速度的积分  二阶龙格库塔法
* @param  quaternion: 原始的姿态
* @param  data      : 设备的角速度
* @retval 积分完后的姿态
*/
void QuaternionInt(double quaternion[4],double data[3] )
{          
  /* 角度弧度转换 */
  data[0]=(data[0])/180.0f*PI;
  data[1]=(data[1])/180.0f*PI;
  data[2]=(data[2])/180.0f*PI;
  
  double dif_quarterion_1[4]={0.0};
  double dif_quarterion_2[4]={0.0};
  double dif_quarterion_3[4]={0.0};
  double dif_quarterion_4[4]={0.0};
  double temp_quarterion[4]={0.0};
  /*根据其阶数不同改变angle的值*/
  for(int grade=1;grade<5;grade++){
    temp_quarterion[0]=quaternion[0];
    temp_quarterion[1]=quaternion[1];
    temp_quarterion[2]=quaternion[2];
    temp_quarterion[3]=quaternion[3];
    switch(grade){
    case 1:
      calculateK(temp_quarterion,dif_quarterion_1,data);
      break;
    case 2:
      temp_quarterion[0]=temp_quarterion[0]+dif_quarterion_1[0]/2.0;
      temp_quarterion[1]=temp_quarterion[1]+dif_quarterion_1[1]/2.0;
      temp_quarterion[2]=temp_quarterion[2]+dif_quarterion_1[2]/2.0;
      temp_quarterion[3]=temp_quarterion[3]+dif_quarterion_1[3]/2.0;
      calculateK(temp_quarterion,dif_quarterion_2,data);
      break;
    case 3:
      temp_quarterion[0]=temp_quarterion[0]+dif_quarterion_2[0]/2.0;
      temp_quarterion[1]=temp_quarterion[1]+dif_quarterion_2[1]/2.0;
      temp_quarterion[2]=temp_quarterion[2]+dif_quarterion_2[2]/2.0;
      temp_quarterion[3]=temp_quarterion[3]+dif_quarterion_2[3]/2.0;
      calculateK(temp_quarterion,dif_quarterion_3,data);
      break;
    case 4:
      temp_quarterion[0]=temp_quarterion[0]+dif_quarterion_3[0];
      temp_quarterion[1]=temp_quarterion[1]+dif_quarterion_3[1];
      temp_quarterion[2]=temp_quarterion[2]+dif_quarterion_3[2];
      temp_quarterion[3]=temp_quarterion[3]+dif_quarterion_3[3];
      calculateK(temp_quarterion,dif_quarterion_4,data);
      break;
    }
  }
  
  quaternion[0]=quaternion[0]+0.166666666*(dif_quarterion_1[0]+2.0*dif_quarterion_2[0]+2.0*dif_quarterion_3[0]+dif_quarterion_4[0]);
  quaternion[1]=quaternion[1]+0.166666666*(dif_quarterion_1[1]+2.0*dif_quarterion_2[1]+2.0*dif_quarterion_3[1]+dif_quarterion_4[1]);
  quaternion[2]=quaternion[2]+0.166666666*(dif_quarterion_1[2]+2.0*dif_quarterion_2[2]+2.0*dif_quarterion_3[2]+dif_quarterion_4[2]);
  quaternion[3]=quaternion[3]+0.166666666*(dif_quarterion_1[3]+2.0*dif_quarterion_2[3]+2.0*dif_quarterion_3[3]+dif_quarterion_4[3]);
}
void QuaternionInt1(double quaternion[4],double data[3] )
{          
	static double old_w[3] ={0,0,0};
  double dif_quarterion_f[4];
	double dif_quarterion_l[4];
	double med_quarterion[4];
	
	dif_quarterion_f[0]=(-quaternion[1]*old_w[0] - quaternion[2]*old_w[1] - quaternion[3]*old_w[2])*0.5f;
	dif_quarterion_f[1]=( quaternion[0]*old_w[0] + quaternion[2]*old_w[2] - quaternion[3]*old_w[1])*0.5f;
	dif_quarterion_f[2]=( quaternion[0]*old_w[1] - quaternion[1]*old_w[2] + quaternion[3]*old_w[0])*0.5f;
	dif_quarterion_f[3]=( quaternion[0]*old_w[2] + quaternion[1]*old_w[1] - quaternion[2]*old_w[0])*0.5f;
	
	med_quarterion[0]=quaternion[0]+dif_quarterion_f[0]*dT;
	med_quarterion[1]=quaternion[1]+dif_quarterion_f[1]*dT;
	med_quarterion[2]=quaternion[2]+dif_quarterion_f[2]*dT;
	med_quarterion[3]=quaternion[3]+dif_quarterion_f[3]*dT; 
  
	dif_quarterion_l[0]=(-med_quarterion[1]*data[0] - med_quarterion[2]*data[1] - med_quarterion[3]*data[2])*0.5f;
	dif_quarterion_l[1]=( med_quarterion[0]*data[0] + med_quarterion[2]*data[2] - med_quarterion[3]*data[1])*0.5f;
	dif_quarterion_l[2]=( med_quarterion[0]*data[1] - med_quarterion[1]*data[2] + med_quarterion[3]*data[0])*0.5f;
	dif_quarterion_l[3]=( med_quarterion[0]*data[2] + med_quarterion[1]*data[1] - med_quarterion[2]*data[0])*0.5f;
	
	
	quaternion[0]=quaternion[0]+0.5f*(dif_quarterion_f[0]+dif_quarterion_l[0])*dT;
	quaternion[1]=quaternion[1]+0.5f*(dif_quarterion_f[1]+dif_quarterion_l[1])*dT;
	quaternion[2]=quaternion[2]+0.5f*(dif_quarterion_f[2]+dif_quarterion_l[2])*dT;
	quaternion[3]=quaternion[3]+0.5f*(dif_quarterion_f[3]+dif_quarterion_l[3])*dT;
	
	for(int i=0;i<3;i++)
		old_w[i]=data[i];
}


int JudgeAcc(void)
{
	float sum[GYRO_NUMBER]={0.f};
	for(int gyro=0;gyro<GYRO_NUMBER;gyro++)
	{
		float X_G,Y_G,Z_G;
		sum[gyro]=sqrt(allPara.ACC_Raw[gyro][0]*allPara.ACC_Raw[gyro][0]+allPara.ACC_Raw[gyro][1]*allPara.ACC_Raw[gyro][1]+allPara.ACC_Raw[gyro][2]*allPara.ACC_Raw[gyro][2]);
		X_G=allPara.ACC_Raw[gyro][0]/sum[gyro];
		Y_G=allPara.ACC_Raw[gyro][1]/sum[gyro];
		Z_G=allPara.ACC_Raw[gyro][2]/sum[gyro];
		/*初始坐标为0,0,g,然后可以通过坐标变换公式轻易推导*/
		allPara.ACC_Angle[gyro][0]= safe_atan2(Y_G,Z_G);
		allPara.ACC_Angle[gyro][1]= safe_asin(-X_G);
		allPara.ACC_Angle[gyro][0]*=57.2957795130823f;
		allPara.ACC_Angle[gyro][1]*=57.2957795130823f;
	}

    return 1;
}	

