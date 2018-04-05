#ifndef __AHRS_H
#define __AHRS_H

#include "ahrs_math.h"
#include "maa_config.h"
//���ʹ���˴�����
//#define MAG_ENABLE

typedef struct {
	int nCount;
	float m_fDeltaTime;				/* һ�����ڵ�ʱ�䣬������ʱ���õĶ���ʱ��				*/

	float m_fGyro[3];					/* �����ǲ���ֵ																	*/
	float m_fAcc[3];					/* ���ٶȼƲ���ֵ																*/
	float m_fMag[3];

	float m_fAccRef[3];      	/* ������������������������ķ���								*/
#ifdef MAG_ENABLE
	float m_fMagRef[3];
#endif

	float m_VarGyro;					/* �����Ƿ���																		*/
	float m_VarAcc;						/* ���ٶȼƵķ���																*/
#ifdef MAG_ENABLE
	float m_VarMag;
#endif
}sensor_data;


/////////////////////////////////////////////////////////////
/**
  * @brief ������Ԫ��
  */
typedef float Quaternion[4];

/**
  * @brief �������˲���ʼ��
  */
void AHRS_Init(sensor_data* pSensorData,
							 euler_angles* pAngle,
							 Quaternion* pQuat);
/**
  * @brief �������˲����±���
  */
void AHRS_Update(sensor_data* pExtSensorData,
								 euler_angles* pExtAngle,
								 Quaternion* pExtQuat);



//////////////////////////////////////////////////////////////
/**
  * @brief Ԥ��P��ֵ
  */
MAA_Matrix_Typedef*		MAA_PropagateP(MAA_Matrix_Typedef* pPoldMat,
																	   MAA_Matrix_Typedef* pStateMat,
																		 MAA_Matrix_Typedef* pQMat,
																		 MAA_Matrix_Typedef* pPnewMat);
/**
  * @brief ���㿨��������
  */
MAA_Matrix_Typedef* MAA_CalculateK(MAA_Matrix_Typedef* pPMat,
																	 MAA_Matrix_Typedef* pHJMat,
																	 MAA_Matrix_Typedef* pRMat,
																	 MAA_Matrix_Typedef* pKMat);
/**
  * @brief ����P��ֵ
  */
MAA_Matrix_Typedef* MAA_UpdateP(MAA_Matrix_Typedef* pPoldMat,
																MAA_Matrix_Typedef* pKMat,
																MAA_Matrix_Typedef* pHJMat,
																MAA_Matrix_Typedef* pPnewMat);


#endif // !__AHRS_H

