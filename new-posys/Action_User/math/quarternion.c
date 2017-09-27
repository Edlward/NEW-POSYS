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
	
  Quarternion dif_quarterion_f;
	Quarternion dif_quarterion_l;
	Quarternion med_quarterion;
	
	dif_quarterion_f.q0=(-quaternion.q1*data.x - quaternion.q2*data.y - quaternion.q3*data.z)*0.5;
	dif_quarterion_f.q1=( quaternion.q0*data.x + quaternion.q2*data.z - quaternion.q3*data.y)*0.5;
	dif_quarterion_f.q2=( quaternion.q0*data.y - quaternion.q1*data.z + quaternion.q3*data.x)*0.5;
	dif_quarterion_f.q3=( quaternion.q0*data.z + quaternion.q1*data.y - quaternion.q2*data.x)*0.5;
	/*#define dT 					   0.005f           //积分的步长5ms*/
	med_quarterion.q0=quaternion.q0+dif_quarterion_f.q0*dT;
	med_quarterion.q1=quaternion.q1+dif_quarterion_f.q1*dT;
	med_quarterion.q2=quaternion.q2+dif_quarterion_f.q2*dT;
	med_quarterion.q3=quaternion.q3+dif_quarterion_f.q3*dT; 
  
	dif_quarterion_l.q0=(-med_quarterion.q1*data.x - med_quarterion.q2*data.y - med_quarterion.q3*data.z)*0.5;
	dif_quarterion_l.q1=( med_quarterion.q0*data.x + med_quarterion.q2*data.z - med_quarterion.q3*data.y)*0.5;
	dif_quarterion_l.q2=( med_quarterion.q0*data.y - med_quarterion.q1*data.z + med_quarterion.q3*data.x)*0.5;
	dif_quarterion_l.q3=( med_quarterion.q0*data.z + med_quarterion.q1*data.y - med_quarterion.q2*data.x)*0.5;
	
	
	quaternion.q0=quaternion.q0+0.5f*(dif_quarterion_f.q0+dif_quarterion_l.q0)*dT;
	quaternion.q1=quaternion.q1+0.5f*(dif_quarterion_f.q1+dif_quarterion_l.q1)*dT;
	quaternion.q2=quaternion.q2+0.5f*(dif_quarterion_f.q2+dif_quarterion_l.q2)*dT;
	quaternion.q3=quaternion.q3+0.5f*(dif_quarterion_f.q3+dif_quarterion_l.q3)*dT;
	
	return quaternion;
}
