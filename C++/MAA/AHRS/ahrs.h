#ifndef __AHRS_H
#define __AHRS_H

#include "ahrs_math.h"
#include "maa_config.h"
//如果使用了磁力计
//#define MAG_ENABLE

typedef struct {
	int nCount;
	float m_fDeltaTime;				/* 一个周期的时间，我们暂时是用的定长时间				*/

	float m_fGyro[3];					/* 陀螺仪测量值																	*/
	float m_fAcc[3];					/* 加速度计测量值																*/
	float m_fMag[3];

	float m_fAccRef[3];      	/* 重力向量，即重力在三个轴的分量								*/
#ifdef MAG_ENABLE
	float m_fMagRef[3];
#endif

	float m_VarGyro;					/* 陀螺仪方差																		*/
	float m_VarAcc;						/* 加速度计的方差																*/
#ifdef MAG_ENABLE
	float m_VarMag;
#endif
}sensor_data;


/////////////////////////////////////////////////////////////
/**
  * @brief 定义四元数
  */
typedef float Quaternion[4];

/**
  * @brief 卡尔曼滤波初始化
  */
void AHRS_Init(sensor_data* pSensorData,
							 euler_angles* pAngle,
							 Quaternion* pQuat);
/**
  * @brief 卡尔曼滤波更新变量
  */
void AHRS_Update(sensor_data* pExtSensorData,
								 euler_angles* pExtAngle,
								 Quaternion* pExtQuat);



//////////////////////////////////////////////////////////////
/**
  * @brief 预测P的值
  */
MAA_Matrix_Typedef*		MAA_PropagateP(MAA_Matrix_Typedef* pPoldMat,
																	   MAA_Matrix_Typedef* pStateMat,
																		 MAA_Matrix_Typedef* pQMat,
																		 MAA_Matrix_Typedef* pPnewMat);
/**
  * @brief 计算卡尔曼增益
  */
MAA_Matrix_Typedef* MAA_CalculateK(MAA_Matrix_Typedef* pPMat,
																	 MAA_Matrix_Typedef* pHJMat,
																	 MAA_Matrix_Typedef* pRMat,
																	 MAA_Matrix_Typedef* pKMat);
/**
  * @brief 更新P的值
  */
MAA_Matrix_Typedef* MAA_UpdateP(MAA_Matrix_Typedef* pPoldMat,
																MAA_Matrix_Typedef* pKMat,
																MAA_Matrix_Typedef* pHJMat,
																MAA_Matrix_Typedef* pPnewMat);


#endif // !__AHRS_H

