#include "config.h"
#include "rotate.h"

extern AllPara_t allPara;

#define Matrix							arm_matrix_instance_f32
#define MatrixInit					arm_mat_init_f32
#define MatrixAdd						arm_mat_add_f32


void MatrixInverse(Matrix * matrix,Matrix * pSrcB)
{
	Matrix matrixTemp;
	float  Array[4]={0.0};
	MatrixInit(&matrixTemp,2,2,Array);
	
	//��Ϊ�����ı�ԭ�����������ｨ��һ��ԭ���󸱱���������
	MatrixAdd(matrix,&matrixTemp,&matrixTemp);
	
	arm_mat_inverse_f32(&matrixTemp,pSrcB);
	
}

void MatrixMuiltiple(Matrix * pSrcA,Matrix * pSrcB,Matrix * pSrcC)
{
	Matrix matrixTempA;
	float  ArrayA[4]={0.0};
	MatrixInit(&matrixTempA,2,2,ArrayA);
	Matrix matrixTempB;
	float  ArrayB[4]={0.0};
	MatrixInit(&matrixTempB,2,2,ArrayB);
	
	//��Ϊ�����ı�ԭ�����������ｨ��һ��ԭ���󸱱���������
	MatrixAdd(pSrcA,&matrixTempA,&matrixTempA);
	
	//��Ϊ�����ı�ԭ�����������ｨ��һ��ԭ���󸱱���������
	MatrixAdd(pSrcB,&matrixTempB,&matrixTempB);
	
	//�ڽ��о��������ʱ��������Ӻͽ��һ�����ͻ������⣬��Ϊ���߹���һƬ�ڴ�
	arm_mat_mult_f32(&matrixTempA,&matrixTempB,pSrcC);
	
}

//�Ƿ�Ҫ����XY
static int isExchangeXY=1;
//Ҫ������ϵ��ת���ٶȣ�
static float roateAngleLevel1=-135.0;

/*ͨ��������ٶȸ��������ٶ�*/
void RotateLevel1(void)
{
	  Matrix exchangeXY;
		Matrix rotate;
	
		float exchangeXY_Array[4]={0,1,1,0};

		float rotateArray[4]={arm_cos_f32(roateAngleLevel1*PI/180.0f),arm_sin_f32(roateAngleLevel1*PI/180.0f),-arm_sin_f32(roateAngleLevel1*PI/180.0f),arm_cos_f32(roateAngleLevel1*PI/180.0f)};
	
		MatrixInit(&exchangeXY,2,2,exchangeXY_Array);
		MatrixInit(&rotate,2,2,rotateArray);
		
		if(isExchangeXY)
			MatrixMuiltiple(&rotate,&exchangeXY,&rotate);
		
		Matrix sendXY;
		Matrix selfXY;
		
		float sendXY_Array[2]={0,0};
		float selfXY_Array[2]={allPara.sDta.posx,allPara.sDta.posy};
		
		MatrixInit(&sendXY,2,1,sendXY_Array);
		MatrixInit(&selfXY,2,1,selfXY_Array);
		
		arm_mat_mult_f32(&rotate,&selfXY,&sendXY);
		
		allPara.talkData.x=sendXY_Array[0];
		allPara.talkData.y=sendXY_Array[1];
}

void SetXY(float x,float y)
{	  
		Matrix exchangeXY;
		Matrix rotate;
	
		float exchangeXY_Array[4]={0,1,1,0};

		float rotateArray[4]={arm_cos_f32(roateAngleLevel1*PI/180.0f),arm_sin_f32(roateAngleLevel1*PI/180.0f),-arm_sin_f32(roateAngleLevel1*PI/180.0f),arm_cos_f32(roateAngleLevel1*PI/180.0f)};
	
		MatrixInit(&exchangeXY,2,2,exchangeXY_Array);
		MatrixInit(&rotate,2,2,rotateArray);
		
		if(isExchangeXY)
			MatrixMuiltiple(&rotate,&exchangeXY,&rotate);
		
		MatrixInverse(&rotate,&rotate);
		
		Matrix sendXY;
		Matrix selfXY;
		
		float sendXY_Array[2]={x,y};
		float selfXY_Array[2]={0,0};
		
		MatrixInit(&sendXY,2,1,sendXY_Array);
		MatrixInit(&selfXY,2,1,selfXY_Array);
		
		arm_mat_mult_f32(&rotate,&sendXY,&selfXY);
		
		allPara.sDta.posx=selfXY_Array[0];
		allPara.sDta.posy=selfXY_Array[1];
}

void UpdateTalkInfo(void)
{
	static float xLast=0.f;
	static float yLast=0.f;
	
	RotateLevel1();
	
	allPara.talkData.angle=allPara.sDta.Result_Angle[2];
	
	allPara.talkData.speedX=(allPara.talkData.x-xLast)*200.f;
	
	allPara.talkData.speedY=(allPara.talkData.y-yLast)*200.f;
	
	allPara.talkData.wz=allPara.GYRO_Real[2];
	
	xLast=allPara.talkData.x;
	
	yLast=allPara.talkData.y;
	
}

