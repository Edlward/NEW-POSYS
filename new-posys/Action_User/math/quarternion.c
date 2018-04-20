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


/*这是惯性导航上的算法，人家是  z  x  y 旋转的，所以x是-90到90*/
/**
* @brief  将四元数转换为欧拉角
* @param  quaternion: 需要转换的四元数
* @retval 四元数对应的欧拉角
*/
void Quaternion_to_Euler(const double quaternion[4],float Rad[3] )
{
  float q0,q1,q2,q3;
  float sum;
  
  q0=quaternion[0];
  q1=quaternion[1];
  q2=quaternion[2];
  q3=quaternion[3];
  
  sum=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0=q0/sum;
  q1=q1/sum;
  q2=q2/sum;
  q3=q3/sum;
	
  Rad[0]= atan2(-2 * q0 * q1 + 2 * q2 * q3,  q3*q3 - q2 * q2 - q1 * q1 +q0 * q0);
  Rad[1]= safe_asin(2.0f*(-q1*q3 + q0*q2));
  Rad[2]=atan2(2*q1*q2+2*q0*q3,-q2*q2-q3*q3+q0*q0+q1*q1);
}

/**
* @brief  将欧拉角转换为四元数
* @param  quaternion: 需要转换的欧拉角
* @retval 四元数对应的四元数
*/

void Euler_to_Quaternion(const float Rad[3],double quaternion[4])
{
	quaternion[0]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[1]=arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)-arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[2]=arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[3]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2)-arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2);
}

void getJacobi(double dif_quarterion[4],const double quaternion[4],const float data[3]){
  dif_quarterion[0]=(-quaternion[1]*data[0] - quaternion[2]*data[1] - quaternion[3]*data[2])*0.5;
  dif_quarterion[1]=( quaternion[0]*data[0] + quaternion[2]*data[2] - quaternion[3]*data[1])*0.5;
  dif_quarterion[2]=( quaternion[0]*data[1] - quaternion[1]*data[2] + quaternion[3]*data[0])*0.5;
  dif_quarterion[3]=( quaternion[0]*data[2] + quaternion[1]*data[1] - quaternion[2]*data[0])*0.5;
};
void  calculateK(double quaternion[4],double dif_quarterion[4],float data[3]){
  
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
void QuaternionInt(double quaternion[4],float data[3] )
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
void QuaternionInt1(double quaternion[4],float data[3] )
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

