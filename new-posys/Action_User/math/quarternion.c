#include "arm_math.h"
#include "figureAngle.h"



/**
  * @brief  将四元数转换为欧拉角
  * @param  quaternion: 需要转换的四元数
  * @retval 四元数对应的欧拉角
  */
three_axis Quaternion_to_Euler(Quarternion quaternion)
{
	three_axis Rad;
	float q0,q1,q2,q3;
	float sum;
		
	q0=quaternion.q0;
	q1=quaternion.q1;
	q2=quaternion.q2;
	q3=quaternion.q3;
	
	sum=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0/sum;
	q1=q1/sum;
	q2=q2/sum;
	q3=q3/sum;
	
	Rad.x= safe_asin(2.0f*(q2*q3 + q0*q1));
	Rad.y= atan2(-2 * q1 * q3 + 2 * q0 * q2,  q3*q3 - q2 * q2 - q1 * q1 +q0 * q0);
	Rad.z= atan2(2*q1*q2-2*q0*q3,q2*q2-q3*q3+q0*q0-q1*q1);
	return Rad;
}

/**
  * @brief  将欧拉角转换为四元数
  * @param  quaternion: 需要转换的欧拉角
  * @retval 四元数对应的四元数
  */
Quarternion Euler_to_Quaternion(three_axis Rad)
{
  Quarternion quaternion;

	quaternion.q0=0.5f*__sqrtf(1+arm_cos_f32(Rad.y)*arm_cos_f32(Rad.z)+arm_cos_f32(Rad.x)*arm_cos_f32(Rad.z)+arm_cos_f32(Rad.x)*arm_cos_f32(Rad.y)+arm_sin_f32(Rad.x)*arm_sin_f32(Rad.y)*arm_sin_f32(Rad.z));
	if(quaternion.q0==0)
	{
		quaternion.q0=1.0;
		quaternion.q1=0.0;
		quaternion.q2=0.0;
		quaternion.q3=0.0;
	}
	else
	{
	  quaternion.q1=(arm_sin_f32(Rad.x)+arm_sin_f32(Rad.y)*arm_sin_f32(Rad.z)+arm_sin_f32(Rad.x)*arm_cos_f32(Rad.y)*arm_cos_f32(Rad.z))/4.f/quaternion.q0;
	  quaternion.q2=(arm_sin_f32(Rad.y)*arm_cos_f32(Rad.z)-arm_cos_f32(Rad.y)*arm_sin_f32(Rad.z)*arm_sin_f32(Rad.x)+arm_sin_f32(Rad.y)*arm_cos_f32(Rad.x))/4.f/quaternion.q0;
	  quaternion.q3=(-arm_cos_f32(Rad.y)*arm_sin_f32(Rad.z)+arm_sin_f32(Rad.y)*arm_cos_f32(Rad.z)*arm_sin_f32(Rad.x)-arm_sin_f32(Rad.z)*arm_cos_f32(Rad.x))/4.f/quaternion.q0;
	}
	
	return quaternion; 
}

void getJacobi(Quarternion *dif_quarterion,Quarternion quaternion,three_axis_d data){
	dif_quarterion->q0=(-quaternion.q1*data.x - quaternion.q2*data.y - quaternion.q3*data.z)*0.5;
	dif_quarterion->q1=( quaternion.q0*data.x + quaternion.q2*data.z - quaternion.q3*data.y)*0.5;
	dif_quarterion->q2=( quaternion.q0*data.y - quaternion.q1*data.z + quaternion.q3*data.x)*0.5;
	dif_quarterion->q3=( quaternion.q0*data.z + quaternion.q1*data.y - quaternion.q2*data.x)*0.5;
};
void  calculateK(const Quarternion quaternion,Quarternion* dif_quarterion,three_axis_d data){
  
	Quarternion  jacobi;
  getJacobi(&jacobi,quaternion,data);
  
  dif_quarterion->q0=jacobi.q0*dT;
	dif_quarterion->q1=jacobi.q1*dT;
	dif_quarterion->q2=jacobi.q2*dT;
	dif_quarterion->q3=jacobi.q3*dT; 
}

/**
  * @brief  针对四元数与角速度的积分  二阶龙格库塔法
  * @param  quaternion: 原始的姿态
  * @param  data      : 设备的角速度
  * @retval 积分完后的姿态
  */
Quarternion QuaternionInt(Quarternion quaternion,three_axis_d data)
{          
	/* 角度弧度转换 */
	data.x=(data.x)/180.0*PI;
	data.y=(data.y)/180.0*PI;
	data.z=(data.z)/180.0*PI;
	
  Quarternion dif_quarterion_1={0.0};
	Quarternion dif_quarterion_2={0.0};
  Quarternion dif_quarterion_3={0.0};
	Quarternion dif_quarterion_4={0.0};
  Quarternion temp_quarterion={0.0};
  /*根据其阶数不同改变angle的值*/
  for(int grade=1;grade<5;grade++){
		temp_quarterion.q0=quaternion.q0;
		temp_quarterion.q1=quaternion.q1;
		temp_quarterion.q2=quaternion.q2;
		temp_quarterion.q3=quaternion.q3;
    switch(grade){
    case 1:
      calculateK(temp_quarterion,&dif_quarterion_1,data);
      break;
    case 2:
			temp_quarterion.q0=temp_quarterion.q0+dif_quarterion_1.q0/2.0;
			temp_quarterion.q1=temp_quarterion.q1+dif_quarterion_1.q1/2.0;
			temp_quarterion.q2=temp_quarterion.q2+dif_quarterion_1.q2/2.0;
			temp_quarterion.q3=temp_quarterion.q3+dif_quarterion_1.q3/2.0;
      calculateK(temp_quarterion,&dif_quarterion_2,data);
      break;
    case 3:
			temp_quarterion.q0=temp_quarterion.q0+dif_quarterion_2.q0/2.0;
			temp_quarterion.q1=temp_quarterion.q1+dif_quarterion_2.q1/2.0;
			temp_quarterion.q2=temp_quarterion.q2+dif_quarterion_2.q2/2.0;
			temp_quarterion.q3=temp_quarterion.q3+dif_quarterion_2.q3/2.0;
      calculateK(temp_quarterion,&dif_quarterion_3,data);
      break;
    case 4:
			temp_quarterion.q0=temp_quarterion.q0+dif_quarterion_3.q0;
			temp_quarterion.q1=temp_quarterion.q1+dif_quarterion_3.q1;
			temp_quarterion.q2=temp_quarterion.q2+dif_quarterion_3.q2;
			temp_quarterion.q3=temp_quarterion.q3+dif_quarterion_3.q3;
      calculateK(temp_quarterion,&dif_quarterion_4,data);
      break;
    }
  }
  
	quaternion.q0=quaternion.q0+0.166666666*(dif_quarterion_1.q0+2.0*dif_quarterion_2.q0+2.0*dif_quarterion_3.q0+dif_quarterion_4.q0);
	quaternion.q1=quaternion.q1+0.166666666*(dif_quarterion_1.q1+2.0*dif_quarterion_2.q1+2.0*dif_quarterion_3.q1+dif_quarterion_4.q1);
	quaternion.q2=quaternion.q2+0.166666666*(dif_quarterion_1.q2+2.0*dif_quarterion_2.q2+2.0*dif_quarterion_3.q2+dif_quarterion_4.q2);
	quaternion.q3=quaternion.q3+0.166666666*(dif_quarterion_1.q3+2.0*dif_quarterion_2.q3+2.0*dif_quarterion_3.q3+dif_quarterion_4.q3);
	
	return quaternion;
}


