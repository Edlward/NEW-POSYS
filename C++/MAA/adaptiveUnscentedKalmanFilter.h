/**
  ******************************************************************************
  * @file    adaptiveUnscentedKalmanFilter.h
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2017.3.19
  * @brief   This file contains the headers of usart.cpp
  ******************************************************************************
  * @attention
  *				下面主要是无迹卡尔曼滤波
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADAPTIVEUNSCENTEDKALMANFILTER_H
#define __ADAPTIVEUNSCENTEDKALMANFILTER_H

/* C&C++ ---------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/	 
/* Exported functions ------------------------------------------------------- */
#ifdef __cplusplus
}
/* Includes ------------------------------------------------------------------*/
#include "action_matrix.h"
#include "vector"
/* Exported class    -------------------------------------------------------- */
class AUKF{
	protected:
		
		float a;
		float b;
		float microParam;	
		
	  float factorR;
		action_matrix* r;
		uint32_t rLen;	
	
		action_matrix Z_pre;
		action_matrix Kk;
		action_matrix P_XZ;
		action_matrix P_R;
	
		action_matrix (*predict)(const action_matrix&,const action_matrix&);
		action_matrix (*measure)(const action_matrix&);
		std::vector<action_matrix> unscentedTrans(const action_matrix&,const action_matrix&);
		action_matrix updateM(std::vector<action_matrix>);
		action_matrix updateC(std::vector<action_matrix>,const action_matrix&,std::vector<action_matrix>,const action_matrix&);
	
		void predictX(const action_matrix&);
		void predictZ(void);
		void estimateX(const action_matrix&);
		void adaptiveR(const action_matrix&);
	
	public:
		action_matrix P;
		action_matrix X;
		action_matrix Q;
		action_matrix QL;	
	  action_matrix R;
		AUKF(float,float,float,
				 const action_matrix&,const action_matrix&,const action_matrix&,const action_matrix&,const action_matrix&,
				 action_matrix (*)(const action_matrix&,const action_matrix&),
				 action_matrix (*)(const action_matrix&),uint32_t len);
		action_matrix filter(const action_matrix&,const	action_matrix&);	
				 
		virtual ~AUKF();
};
#endif
/* Exported function     ------------------------------------------------------- */
#endif

/******************* (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
