#include "attitudeAngle.h"
#include "action_matrix.h"



/**
  * @brief  通过姿态四元数转换得到坐标矩阵,这矩阵表示的是从b系到R系间的坐标转换
  * @param  旋转四元数
  * @retval 坐标转换矩阵
  * @author Lxy
**/
action_matrix getTransMatrix(const action_matrix& value)
{
	action_matrix transMatrix(3,3);
	
	transMatrix[0][0]=1-2*value[2][0]*value[2][0]-2*value[3][0]*value[3][0];
	transMatrix[0][1]=2*value[1][0]*value[2][0]-2*value[0][0]*value[3][0];
	transMatrix[0][2]=2*value[1][0]*value[3][0]+2*value[0][0]*value[2][0];
	
	transMatrix[1][0]=2*value[1][0]*value[2][0]+2*value[0][0]*value[3][0];
	transMatrix[1][1]=1-2*value[1][0]*value[1][0]-2*value[3][0]*value[3][0];
	transMatrix[1][2]=2*value[2][0]*value[3][0]-2*value[0][0]*value[1][0];
	
	transMatrix[2][0]=2*value[1][0]*value[3][0]-2*value[0][0]*value[2][0];
	transMatrix[2][1]=2*value[2][0]*value[3][0]+2*value[0][0]*value[1][0];
	transMatrix[2][2]=1-2*value[1][0]*value[1][0]-2*value[2][0]*value[2][0];
	
	return transMatrix;
}
action_matrix integral(const action_matrix& value,const action_matrix& val,float stepSize)
{
	action_matrix Mw(4,4);
	
	float sum=0;
	for(uint8_t i=0;i<4;i++)
	{
		sum+=value[i][0]*value[i][0];
	}
	sum=sqrt(sum);
	for(uint8_t i=0;i<4;i++)
	{
		value[i][0]=value[i][0]/sum;
	}	
	
	
	Mw[0][0]=0;					Mw[0][1]=-val[0][0]; Mw[0][2]=-val[1][0]; Mw[0][3]=-val[2][0];
	Mw[1][0]=val[0][0];	Mw[1][1]=0;			 		 Mw[1][2]= val[2][0]; Mw[1][3]=-val[1][0];
	Mw[2][0]=val[1][0];	Mw[2][1]=-val[2][0]; Mw[2][2]= 0; 				Mw[2][3]= val[0][0];
	Mw[3][0]=val[2][0];	Mw[3][1]= val[1][0]; Mw[3][2]=-val[0][0]; Mw[3][3]= 0;
	
	static action_matrix dif_quarterion_f(4,1);
	static action_matrix med_quarterion(4,1);
	static action_matrix dif_quarterion_l(4,1);
	
	dif_quarterion_f=Mw*value*0.5f;
	med_quarterion=value+dif_quarterion_f*stepSize;
	dif_quarterion_l=Mw*med_quarterion*0.5f;
	return (value+0.5f*(dif_quarterion_f+dif_quarterion_l)*stepSize);
}



