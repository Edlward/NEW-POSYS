#include "arm_math.h"
#include "leastSquare.h"

#define ARRAY (MaxCoeff+1)
/*�¶ȳ���10���������*/
extern uint32_t *chartNum;

float coefficient[MaxCoeff]={0};
int PCSolve(float MatrixA[ARRAY][ARRAY], float MatrixB[ARRAY], float* result);

//float TemSumByPower(float data[Table_Num],int power){
//	float sum=0.f;
//	for(int i=0;i<Table_Num;i++){
//		if(data[i]!=0.f&&chartNum[i]>=LEASTNUM*2)
//			sum+=pow((30+0.1f*i)/10.f,power);
//	}
//	return sum;
//}

//float ResultSumByPower(float data[Table_Num],int power){
//	float sum=0.f;
//	for(int i=0;i<Table_Num;i++){
//		if(data[i]!=0.f&&chartNum[i]>=LEASTNUM*2)
//			sum+=pow((30+0.1*i)/10.f,power)*data[i];
//	}
//	return sum;
//}

static float fittingCoeff[MaxCoeff+1]={0};

void SquareFitting(float data[Table_Num]){
	
	static float rightArray[ARRAY]={0};
	
	static float leftArray[ARRAY][ARRAY]={0};
	
	static float resultArray[ARRAY]={0};
	
	for(int i=0;i<ARRAY;i++){
		for(int j=0;j<ARRAY;j++){
//			leftArray[i][j]=TemSumByPower(data,i+j);
		}
	}
	
	for(int i=0;i<ARRAY;i++){
//		resultArray[i]=ResultSumByPower(data,i);
	}
	
	PCSolve(leftArray,resultArray,rightArray);
	for(int i=0;i<ARRAY;i++){
		fittingCoeff[i]=rightArray[i];
	}
}

double FitResult(float tem){
	double result=0.0;
	for(int i=0;i<MaxCoeff+1;i++){
		result+=fittingCoeff[i]*pow(tem/10.f,i);
	}
	return result;
	
}

int Max(float* data, int i)
{
	int order = i;
	/*�ֻ������ж������������*/
	while (i < ARRAY - 1)
	{
		if (fabs(*(data + order))<fabs(*(data + i + 1)))
			order = i + 1;
		i++;
	}
	return order;
}
void ChangeROW(float matrixA[][ARRAY], float* matrixB, int k)
{
	/*��ȡ���Ԫ����������Ҫ����׼��*/
	float tmpMatrix[ARRAY] = { 0 };
	/*��ȡ��k��Ԫ��*/
	for (int j = 0; j < ARRAY; j++)
	{
		tmpMatrix[j] = matrixA[j][k];
	}
	/*�ӵ�k�п�ʼ,��ȡ���Ԫ��������*/
	int m = Max(tmpMatrix, k);
	/*�����matrixA[i][i]�������Ԫ��*/
	if (k != m)
	{
		/*�Ե�i�к͵�m�н��н���*/
		float midMatrix[ARRAY] = { 0 };
		float mid = 0;
		for (int j = 0; j < ARRAY; j++)
		{
			midMatrix[j] = matrixA[k][j];
			matrixA[k][j] = matrixA[m][j];
			matrixA[m][j] = midMatrix[j];
			mid = matrixB[k];
			matrixB[k] = matrixB[m];
			matrixB[m] = mid;
		}
	}
}
void Eliminate(float matrixA[][ARRAY], float* matrixB, int k)
{
	for (int i = k + 1; i < ARRAY; i++)
	{
		float l = matrixA[i][k] / matrixA[k][k];
		for (int m = k; m < ARRAY; m++)
			matrixA[i][m] = matrixA[i][m] - l*matrixA[k][m];
		*(matrixB + i) = *(matrixB + i) - l* (*(matrixB + k));
	}
	
}
/*����Ԫ��Ԫ��*/
int PCSolve(float MatrixA[ARRAY][ARRAY], float MatrixB[ARRAY], float* result)
{
	float matrixA[ARRAY][ARRAY] = { 0.f };
	float matrixB[ARRAY] = { 0.f };

	for (int i = 0; i < ARRAY; i++)
		for (int j = 0; j < ARRAY; j++)
			matrixA[i][j] = MatrixA[i][j];

	for (int j = 0; j < ARRAY; j++)
		matrixB[j] = MatrixB[ARRAY-j-1];

	/*�ӵ�1�е���ARRAY��*/
	for (int k = 0; k < ARRAY; k++)
	{
		/*����Ԫ,ѡ���*/
		ChangeROW(matrixA, matrixB, k);
		//��Ԫ
		Eliminate(matrixA, matrixB, k);
		/*�ж���Ԫ���Ƿ�Ϊ0*/
		if (matrixA[k][k] == 0.0f)
			return 0;
	}
	/*���д������*/
	*(result + ARRAY - 1) = matrixB[ARRAY - 1] / matrixA[ARRAY - 1][ARRAY - 1];
	for (int k = ARRAY - 2; k >= 0; k--)
	{
		float tempB[ARRAY] ;
		for (int i = 0; i < ARRAY; i++)
			tempB[i] = matrixB[i];
		for (int m = k + 1; m < ARRAY; m++)
		{
			tempB[k] = tempB[k] - matrixA[k][m] * (*(result + m));
		}
		*(result + k) = tempB[k] / matrixA[k][k];
	}
	return 1;
}


