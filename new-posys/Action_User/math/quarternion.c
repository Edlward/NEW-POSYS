//#include "arm_math.h"
//#include "leastSquare.h"

//#define ARRAY (MaxCoeff+1)
///*温度除以10，进行拟合*/
//extern uint32_t *chartNum;

//float coefficient[MaxCoeff]={0};
//int PCSolve(float MatrixA[ARRAY][ARRAY], float MatrixB[ARRAY], float* result);

//float TemSumByPower(float data[TempTable_Num],int power){
//	float sum=0.f;
//	for(int i=0;i<TempTable_Num;i++){
//		if(data[i]!=0.f&&chartNum[i]>=LEASTNUM*2)
//			sum+=pow((30+0.1f*i)/10.f,power);
//	}
//	return sum;
//}

//float ResultSumByPower(float data[TempTable_Num],int power){
//	float sum=0.f;
//	for(int i=0;i<TempTable_Num;i++){
//		if(data[i]!=0.f&&chartNum[i]>=LEASTNUM*2)
//			sum+=pow((30+0.1*i)/10.f,power)*data[i];
//	}
//	return sum;
//}

//static float fittingCoeff[MaxCoeff+1]={0};

//void SquareFitting(float data[TempTable_Num]){
//	
//	static float rightArray[ARRAY]={0};
//	
//	static float leftArray[ARRAY][ARRAY]={0};
//	
//	static float resultArray[ARRAY]={0};
//	
//	for(int i=0;i<ARRAY;i++){
//		for(int j=0;j<ARRAY;j++){
//			leftArray[i][j]=TemSumByPower(data,i+j);
//		}
//	}
//	
//	for(int i=0;i<ARRAY;i++){
//		resultArray[i]=ResultSumByPower(data,i);
//	}
//	
//	PCSolve(leftArray,resultArray,rightArray);
//	for(int i=0;i<ARRAY;i++){
//		fittingCoeff[i]=rightArray[i];
//	}
//}

//double FitResult(float tem){
//	double result=0.0;
//	for(int i=0;i<MaxCoeff+1;i++){
//		result+=fittingCoeff[i]*pow(tem/10.f,i);
//	}
//	return result;
//	
//}

//int Max(float* data, int i)
//{
//	int order = i;
//	/*轮换进行判断输出最大行序号*/
//	while (i < ARRAY - 1)
//	{
//		if (fabs(*(data + order))<fabs(*(data + i + 1)))
//			order = i + 1;
//		i++;
//	}
//	return order;
//}
//void ChangeROW(float matrixA[][ARRAY], float* matrixB, int k)
//{
//	/*获取最大元素所在行需要做的准备*/
//	float tmpMatrix[ARRAY] = { 0 };
//	/*获取第k列元素*/
//	for (int j = 0; j < ARRAY; j++)
//	{
//		tmpMatrix[j] = matrixA[j][k];
//	}
//	/*从第k行开始,获取最大元素所在行*/
//	int m = Max(tmpMatrix, k);
//	/*如果第matrixA[i][i]不是最大元素*/
//	if (k != m)
//	{
//		/*对第i行和第m行进行交换*/
//		float midMatrix[ARRAY] = { 0 };
//		float mid = 0;
//		for (int j = 0; j < ARRAY; j++)
//		{
//			midMatrix[j] = matrixA[k][j];
//			matrixA[k][j] = matrixA[m][j];
//			matrixA[m][j] = midMatrix[j];
//			mid = matrixB[k];
//			matrixB[k] = matrixB[m];
//			matrixB[m] = mid;
//		}
//	}
//}
//void Eliminate(float matrixA[][ARRAY], float* matrixB, int k)
//{
//	for (int i = k + 1; i < ARRAY; i++)
//	{
//		float l = matrixA[i][k] / matrixA[k][k];
//		for (int m = k; m < ARRAY; m++)
//			matrixA[i][m] = matrixA[i][m] - l*matrixA[k][m];
//		*(matrixB + i) = *(matrixB + i) - l* (*(matrixB + k));
//	}
//	
//}
///*列主元消元法*/
//int PCSolve(float MatrixA[ARRAY][ARRAY], float MatrixB[ARRAY], float* result)
//{
//	float matrixA[ARRAY][ARRAY] = { 0.f };
//	float matrixB[ARRAY] = { 0.f };

//	for (int i = 0; i < ARRAY; i++)
//		for (int j = 0; j < ARRAY; j++)
//			matrixA[i][j] = MatrixA[i][j];

//	for (int j = 0; j < ARRAY; j++)
//		matrixB[j] = MatrixB[ARRAY-j-1];

//	/*从第1列到第ARRAY列*/
//	for (int k = 0; k < ARRAY; k++)
//	{
//		/*列主元,选最大*/
//		ChangeROW(matrixA, matrixB, k);
//		//消元
//		Eliminate(matrixA, matrixB, k);
//		/*判断主元素是否为0*/
//		if (matrixA[k][k] == 0.0f)
//			return 0;
//	}
//	/*进行带换求解*/
//	*(result + ARRAY - 1) = matrixB[ARRAY - 1] / matrixA[ARRAY - 1][ARRAY - 1];
//	for (int k = ARRAY - 2; k >= 0; k--)
//	{
//		float tempB[ARRAY] ;
//		for (int i = 0; i < ARRAY; i++)
//			tempB[i] = matrixB[i];
//		for (int m = k + 1; m < ARRAY; m++)
//		{
//			tempB[k] = tempB[k] - matrixA[k][m] * (*(result + m));
//		}
//		*(result + k) = tempB[k] / matrixA[k][k];
//	}
//	return 1;
//}


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
  Rad.z=atan2(2*q1*q2-2*q0*q3,q2*q2-q3*q3+q0*q0-q1*q1);
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
  if(quaternion.q0==0.0)
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
Quarternion QuaternionInt1(Quarternion quaternion,three_axis_d data)
{          
	static three_axis_d old_w={0,0,0};
  Quarternion dif_quarterion_f;
	Quarternion dif_quarterion_l;
	Quarternion med_quarterion;
	
	dif_quarterion_f.q0=(-quaternion.q1*old_w.x - quaternion.q2*old_w.y - quaternion.q3*old_w.z)*0.5f;
	dif_quarterion_f.q1=( quaternion.q0*old_w.x + quaternion.q2*old_w.z - quaternion.q3*old_w.y)*0.5f;
	dif_quarterion_f.q2=( quaternion.q0*old_w.y - quaternion.q1*old_w.z + quaternion.q3*old_w.x)*0.5f;
	dif_quarterion_f.q3=( quaternion.q0*old_w.z + quaternion.q1*old_w.y - quaternion.q2*old_w.x)*0.5f;
	
	med_quarterion.q0=quaternion.q0+dif_quarterion_f.q0*dT;
	med_quarterion.q1=quaternion.q1+dif_quarterion_f.q1*dT;
	med_quarterion.q2=quaternion.q2+dif_quarterion_f.q2*dT;
	med_quarterion.q3=quaternion.q3+dif_quarterion_f.q3*dT; 
  
	dif_quarterion_l.q0=(-med_quarterion.q1*data.x - med_quarterion.q2*data.y - med_quarterion.q3*data.z)*0.5f;
	dif_quarterion_l.q1=( med_quarterion.q0*data.x + med_quarterion.q2*data.z - med_quarterion.q3*data.y)*0.5f;
	dif_quarterion_l.q2=( med_quarterion.q0*data.y - med_quarterion.q1*data.z + med_quarterion.q3*data.x)*0.5f;
	dif_quarterion_l.q3=( med_quarterion.q0*data.z + med_quarterion.q1*data.y - med_quarterion.q2*data.x)*0.5f;
	
	
	quaternion.q0=quaternion.q0+0.5f*(dif_quarterion_f.q0+dif_quarterion_l.q0)*dT;
	quaternion.q1=quaternion.q1+0.5f*(dif_quarterion_f.q1+dif_quarterion_l.q1)*dT;
	quaternion.q2=quaternion.q2+0.5f*(dif_quarterion_f.q2+dif_quarterion_l.q2)*dT;
	quaternion.q3=quaternion.q3+0.5f*(dif_quarterion_f.q3+dif_quarterion_l.q3)*dT;
	
	
	old_w=data;
	return quaternion;
}

