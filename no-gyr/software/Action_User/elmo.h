#ifndef __ELMO_H
#define __ELMO_H

#include "stm32f4xx.h"

//ELMO������CAN�㲥ID��
#define ELMO_BROADCAST_ID  (0x000)
//ELMO����������ID��ַ 
#define ELMO_DEVICE_BASEID (0x300)
//ELMO����������ID��ַ
#define SDO_RESPONSE_COB_ID_BASE 0x280

/******************����������ģʽ************************/
#define TORQUE_CONTROL_MODE   (1)
#define SPEED_CONTROL_MODE    (2)
#define MICRO_STEPPER_MODE    (3)
#define DUAL_POSITION_MODE    (4)
#define SINGLE_POSITION_MODE  (5)

/*********************λ�û�����ģʽ**********************/
#define ABSOLUTE_MODE (0)
#define RELATIVE_MODE (1)


/*******************************��������������************************************/
/**
* @brief  Elmo��������ʼ��
* @param  CANx����ʹ�õ�CANͨ�����
* @author ACTION
*/
void ElmoInit(CAN_TypeDef* CANx);

/**
* @brief  ���ʹ�ܣ�ͨ�磩
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
* @note ELMO������Ĭ�ϳ�ʼ״̬Ϊ���ʧ�ܣ�ʹ�õ��ʱ��Ҫ�������ʹ��
*       ����������������Ҫ�ڵ��ʧ��״̬�²ſ�������
*/
void MotorOn(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ���ʧ�ܣ��ϵ磩
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
*/
void MotorOff(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  �������ٶȻ���ʼ��
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  acc�����ٶȣ���λ������ÿ���η���
* @param  dec�����ٶȣ���λ������ÿ���η���
* @author ACTION
* @note ���ٶȻ���ʼ����ſ���ʹ�ܵ������
*/
void VelLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

/**
* @brief  ������λ�û���ʼ��
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  acc�����ٶȣ���λ������ÿ���η���
* @param  dec�����ٶȣ���λ������ÿ���η���
* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
* @author ACTION
* @note ��λ�û���ʼ����ſ���ʹ�ܵ������
*/
void PosLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec,uint32_t vel);

/**
* @brief  ����ٶȿ���
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
* @author ACTION
*/
void VelCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel);

/**
* @brief  ���λ�ÿ���
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  posMode: λ�û�����ģʽ����Χ��
				ABSOLUTE_MODE: ����λ��ģʽ
				RELATIVE_MODE: ���λ��ģʽ
* @param  pos:λ�������λ�����壬��Χ�����λ�����Ƶ���Сλ������
* @author ACTION
*/
void PosCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos);

/**
* @brief  ��������������ģʽ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  unitMode������������ģʽ����Χ��
			TORQUE_CONTROL_MODE�����ؿ���ģʽ���ڸ�ģʽ�¿���ִ��TC��������
			SPEED_CONTROL_MODE���ٶȿ���ģʽ���ڸ�ģʽ��ͨ������JVֵ�����ٶ�
			MICRO_STEPPER_MODE��ֱ���������ʹ�ø�ģʽ
			DUAL_POSITION_MODE��˫λ�ñջ�ģʽ
			SINGLE_POSITION_MODE����λ�ñջ�ģʽ���ڸ�ģʽ�¿�������PA��PR��JV��PT��PVT�˶�
* @author ACTION
* @note ֻ���ڵ��ʧ��ʱ�������øò���
*/
void SetUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t unitMode);
/**
* @brief  �����������Ƿ���������ƽ��
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  smoothFactor���Ƿ�ƽ������Χ��0~100����λ�����룩
* @author ACTION
* @note ��ֻ���ڵ��ʧ��ʱ��������
*/
void SetSmoothFactor(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t smoothFactor);

/**
* @brief  ���ü��ٶ�����ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  acc�����ٶȣ���λ������ÿ���η���
* @param  dec�����ٶȣ���λ������ÿ���η���
* @author ACTION
* @note
*/
void SetAccAndDec(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

/**
* @brief  ���õ���ٶ�����
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  upperLimit���������ٶ����ƣ���λ������ÿ�룬��Χ��0~80,000,000
* @param  lowerLimit���������ٶ����ƣ���λ������ÿ�룬��Χ��-80,000,000~0
* @author ACTION
* @note��ֻ���ڵ��ʧ��ʱ�������øò���
*/
void SetVelLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  ���õ��λ������
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  upperLimit��������λ�����ƣ���λ�����壬��Χ��-2^31~2^31-1
* @param  lowerLimit��������λ�����ƣ���λ�����壬��Χ��-2^31~upperLimit
* @author ACTION
* @note��ֻ���ڵ��ʧ��ʱ�������øò���,upperLimit�������lowerLimit
*/
void SetPosLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  ����λ�ü�¼��Χ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  upperLimit��������λ�����ƣ���λ�����壬��Χ��-10^9~10^9
* @param  lowerLimit��������λ�����ƣ���λ�����壬��Χ��-10^9~10^9
* @author ACTION
* @note��ֻ���ڵ��ʧ��ʱ�������øò���,upperLimit�������lowerLimit��upperLimit-lowerLimit�������Ϊż��
*/
void SetPosCountingRange(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  ����λ�û���������ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
* @author ACTION
* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
*/
void SetPosLoopVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel);

/**
* @brief  ���������ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
* @author ACTION
* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
*/
void SetJoggingVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel);

/**
* @brief  ����λ�û�����
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  posMode: λ�û�����ģʽ����Χ��
				ABSOLUTE_MODE: ����λ��ģʽ
				RELATIVE_MODE: ���λ��ģʽ
* @param  pos:λ�������λ�����壬��Χ�����λ�����Ƶ���Сλ������
* @author ACTION
* @note��λ�������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
*/
void SendPosCmd(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos);

/**
* @brief  ��ʼ��һ���˶�����
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
* @note��
*/
void BeginMotion(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**********************************��ȡ��������������*************************************/

/**
* @brief  ��ȡ�����ѹ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
* @note��
*/
void ReadActualVoltage(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ�������
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x80005149
*/
void ReadActualCurrent(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ���λ��
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00005850
*/
void ReadActualPos(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ����ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00005856
*/
void ReadActualVel(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ�������¶�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00014954
*/
void ReadActualTemperature(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ�������������Ʊ�־λ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x0000434C
*/
void ReadCurrentLimitFlag(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ�������ٶ�������������ٶ���ʵ���ٶȵ���
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��
*/
void ReadVelocityError(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ������������ٶ�����ֵ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00025644
*/
void ReadCommandVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ������յ����ٶ�����
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x0000564A
*/
void ReadJoggingVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ���ģʽ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��
*/
void ReadUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ������RM״̬
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��
*/
void ReadReferenceMode(CAN_TypeDef* CANx, uint8_t ElmoNum);

/**
* @brief  ��ȡ�������������
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x0000464D
*/
void ReadMotorFailure(CAN_TypeDef* CANx, uint8_t ElmoNum);

#endif


