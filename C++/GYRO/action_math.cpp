/**
  ******************************************************************************
  * @file    action_math.cpp  
  * @author  lxy
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "action_math.h"
#include "action_matrix.h"
#include "math.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static action_matrix *BasicFittingA;
static action_matrix *BasicFittingY;
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  最小二乘法选择阶数
  * @param  num: 阶数
  * @retval none
  */
void BasicFitting_th(uint8_t num)
{
	BasicFittingA=new action_matrix(num,num,MATRIX_ZERO);
	BasicFittingY=new action_matrix(num,3,MATRIX_ZERO);
}
/**
  * @brief  最小二乘需要拟合的点
  * @param  val_x: x坐标
  * @param  val_y: y坐标
  * @retval none
  */
void ordata_input(float val_x,float val_y,float val_y1,float val_y2)
{
	for(uint8_t row=0;row<(BasicFittingA->get_row());row++)
	{
		for(uint8_t column=0;column<(BasicFittingA->get_row());column++)
		{
			/*x的(row+column)次幂的累加和,得到2*2矩阵*/
			BasicFittingA->set_data(row,column,BasicFittingA->get_data(row,column)+pow(val_x,row+column));
		}
		/*x的row次幂和y的乘积累和,得到2*3矩阵*/
		BasicFittingY->set_data(row,0,BasicFittingY->get_data(row,0)+pow(val_x,row)*val_y);
		BasicFittingY->set_data(row,1,BasicFittingY->get_data(row,1)+pow(val_x,row)*val_y1);
		BasicFittingY->set_data(row,2,BasicFittingY->get_data(row,2)+pow(val_x,row)*val_y2);
	}
}
/**
  * @brief  获取最小二乘法的结果
  * @param  none
  * @retval 最小二乘法拟合结果中的多项式系数
  */
float **BasicFittingCal(void)
{
	action_matrix result(BasicFittingA->get_row(),3);
	/*
	float **re=new float *[2];
	创建一个指向指针的指针
	*/
	float **re=new float *[BasicFittingA->get_row()];
	/*
	这个二维数组有两行,三列.都是我们动态分配的
	*/
	for(uint8_t i=0;i<BasicFittingA->get_row();i++)
	{
		re[i]=new float[3];
	}
	/*求解最小二乘法*/
	result=(~(*BasicFittingA))*(*BasicFittingY);
	
	BasicFittingA->delete_data();
	BasicFittingY->delete_data();
	
	delete BasicFittingA;
	delete BasicFittingY;
	
	for(uint8_t i=0;i<result.get_row();i++)
	{
		for(uint8_t j=0;j<3;j++)
		re[i][j]=result.get_data(i,j);
	}
	result.delete_data();
	//既然return出去了,为什么不delete?
	return re;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
