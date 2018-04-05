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
  * @brief  ��С���˷�ѡ�����
  * @param  num: ����
  * @retval none
  */
void BasicFitting_th(uint8_t num)
{
	BasicFittingA=new action_matrix(num,num,MATRIX_ZERO);
	BasicFittingY=new action_matrix(num,3,MATRIX_ZERO);
}
/**
  * @brief  ��С������Ҫ��ϵĵ�
  * @param  val_x: x����
  * @param  val_y: y����
  * @retval none
  */
void ordata_input(float val_x,float val_y,float val_y1,float val_y2)
{
	for(uint8_t row=0;row<(BasicFittingA->get_row());row++)
	{
		for(uint8_t column=0;column<(BasicFittingA->get_row());column++)
		{
			/*x��(row+column)���ݵ��ۼӺ�,�õ�2*2����*/
			BasicFittingA->set_data(row,column,BasicFittingA->get_data(row,column)+pow(val_x,row+column));
		}
		/*x��row���ݺ�y�ĳ˻��ۺ�,�õ�2*3����*/
		BasicFittingY->set_data(row,0,BasicFittingY->get_data(row,0)+pow(val_x,row)*val_y);
		BasicFittingY->set_data(row,1,BasicFittingY->get_data(row,1)+pow(val_x,row)*val_y1);
		BasicFittingY->set_data(row,2,BasicFittingY->get_data(row,2)+pow(val_x,row)*val_y2);
	}
}
/**
  * @brief  ��ȡ��С���˷��Ľ��
  * @param  none
  * @retval ��С���˷���Ͻ���еĶ���ʽϵ��
  */
float **BasicFittingCal(void)
{
	action_matrix result(BasicFittingA->get_row(),3);
	/*
	float **re=new float *[2];
	����һ��ָ��ָ���ָ��
	*/
	float **re=new float *[BasicFittingA->get_row()];
	/*
	�����ά����������,����.�������Ƕ�̬�����
	*/
	for(uint8_t i=0;i<BasicFittingA->get_row();i++)
	{
		re[i]=new float[3];
	}
	/*�����С���˷�*/
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
	//��Ȼreturn��ȥ��,Ϊʲô��delete?
	return re;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/