#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"


/*******************************��������������************************************/
/**
* @brief  Elmo��������ʼ��
* @param  CANx����ʹ�õ�CANͨ�����
* @author ACTION
*/
void ElmoInit(CAN_TypeDef* CANx)
{
	uint32_t data[1][2]={0x00000001,00000000};
	CAN_TxMsg(CANx,ELMO_BROADCAST_ID,(uint8_t*)&data[0],8);
}

/**
* @brief  ���ʹ�ܣ�ͨ�磩
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
* @note ELMO������Ĭ�ϳ�ʼ״̬Ϊ���ʧ�ܣ�ʹ�õ��ʱ��Ҫ�������ʹ��
*       ����������������Ҫ�ڵ��ʧ��״̬�²ſ�������
*/
void MotorOn(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	//��һ��������MO����ڶ���������1�����ʹ�ܣ�ͨ�磩
	uint32_t data[1][2]={
							0x00004F4D,0x00000001,
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	//Ϊ���ͽṹ�帳ֵ 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted

	TxMessage.DLC=8;

 	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	//��������
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//�ȴ����ͳɹ�
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//������Ӧ�����쳣����
			
		}
	}
}

/**
* @brief  ���ʧ�ܣ��ϵ磩
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
*/
void MotorOff(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	//��һ�����ݷ���MO����ڶ������ݷ���0�����ʧ�ܣ��ϵ磩
	uint32_t data[1][2]={
						0x00004F4D,0x00000000,      //MO  0
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//�ȴ����ͳɹ�
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//������Ӧ�����쳣����
		
		}
	}

}

/**
* @brief  �������ٶȻ���ʼ��
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  acc�����ٶȣ���λ������ÿ���η���
* @param  dec�����ٶȣ���λ������ÿ���η���
* @author ACTION
* @note ���ٶȻ���ʼ����ſ���ʹ�ܵ������
*/
void VelLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
{
	SetUnitMode(CANx, ElmoNum, SPEED_CONTROL_MODE);
	
	SetSmoothFactor(CANx, ElmoNum, 0);
	
	SetAccAndDec(CANx, ElmoNum, acc, dec);
}

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
void PosLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec,uint32_t vel)
{
	SetUnitMode(CANx, ElmoNum, SINGLE_POSITION_MODE);
	
	SetSmoothFactor(CANx, ElmoNum, 0);

	SetAccAndDec(CANx, ElmoNum, acc, dec);

	SetPosLoopVel(CANx, ElmoNum, vel);	
}

/**
* @brief  ����ٶȿ���
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
* @author ACTION
*/
void VelCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel)
{
	SetJoggingVel(CANx, ElmoNum, vel);
	
	BeginMotion(CANx, ElmoNum);
}

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
void PosCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos)
{
	SendPosCmd(CANx, ElmoNum, posMode, pos);
	
	BeginMotion(CANx, ElmoNum);	
}

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
void SetUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t unitMode)
{

	//��һ�����ݷ���UM����ڶ������ݷ���ģʽ
	uint32_t data[1][2]={
						0x00004D55,0x00000000,      //UM
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = unitMode;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//�ȴ����ͳɹ�
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//������Ӧ�����쳣����
		
		}
	}
}

/**
* @brief  �����������Ƿ���������ƽ��
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  smoothFactor���Ƿ�ƽ������Χ��0~100����λ�����룩
* @author ACTION
* @note ��ֻ���ڵ��ʧ��ʱ��������
*/
void SetSmoothFactor(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t smoothFactor)
{
	//��һ�����ݷ���SF����ڶ������ݷ���SF����ֵ
	uint32_t data[1][2]={
						0x00004653,0x00000000,      //SF
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = smoothFactor;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//�ȴ����ͳɹ�
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//������Ӧ�����쳣����
		
		}
	}
}

/**
* @brief  ���ü��ٶ�����ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  acc�����ٶȣ���λ������ÿ���η���
* @param  dec�����ٶȣ���λ������ÿ���η���
* @author ACTION
* @note
*/
void SetAccAndDec(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
{
	//��һ�����ݷ���AC\DC����ڶ������ݷ�������ֵ
	uint32_t data[2][2]={
							0x00004341,0x00000000,		//AC
							0x00004344,0x00000000		//DC
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = acc;
	data[1][1] = dec;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//�ȴ����ͳɹ�
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//������Ӧ�����쳣����
			
			}
		}
	}
}

/**
* @brief  ���õ���ٶ�����
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  upperLimit���������ٶ����ƣ���λ������ÿ�룬��Χ��0~80,000,000
* @param  lowerLimit���������ٶ����ƣ���λ������ÿ�룬��Χ��-80,000,000~0
* @author ACTION
* @note��ֻ���ڵ��ʧ��ʱ�������øò���
*/
void SetVelLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit)
{
	//��һ�����ݷ���VH[2]\VL[2]����ڶ������ݷ�������ֵ
	uint32_t data[2][2]={
							0x00024856,0x00000000,		//VH[2]
							0x00024C56,0x00000000		//VL[2]
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = upperLimit;
	data[1][1] = lowerLimit;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//�ȴ����ͳɹ�
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//������Ӧ�����쳣����
			
			}
		}
	}
}

/**
* @brief  ���õ��λ������
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  upperLimit��������λ�����ƣ���λ�����壬��Χ��-2^31~2^31-1
* @param  lowerLimit��������λ�����ƣ���λ�����壬��Χ��-2^31~upperLimit
* @author ACTION
* @note��ֻ���ڵ��ʧ��ʱ�������øò���,upperLimit�������lowerLimit
*/
void SetPosLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit)
{
	//��һ�����ݷ���VH[3]\VL[3]����ڶ������ݷ�������ֵ
	uint32_t data[2][2]={
							0x00034856,0x00000000,		//VH[3]
							0x00034C56,0x00000000		//VL[3]
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = upperLimit;
	data[1][1] = lowerLimit;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//�ȴ����ͳɹ�
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//������Ӧ�����쳣����
			
			}
		}
	}
}

/**
* @brief  ����λ�ü�¼��Χ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  upperLimit��������λ�����ƣ���λ�����壬��Χ��-10^9~10^9
* @param  lowerLimit��������λ�����ƣ���λ�����壬��Χ��-10^9~10^9
* @author ACTION
* @note��ֻ���ڵ��ʧ��ʱ�������øò���,upperLimit�������lowerLimit��upperLimit-lowerLimit�������Ϊż��
*/
void SetPosCountingRange(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit)
{
	//��һ�����ݷ���VH[3]\VL[3]����ڶ������ݷ�������ֵ
	uint32_t data[2][2]={
							0x00024D58,0x00000000,    //XM[2]
							0x00014D58,0x00000000    //XM[1]
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = upperLimit;
	data[1][1] = lowerLimit;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//�ȴ����ͳɹ�
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//������Ӧ�����쳣����
			
			}
		}
	}
}

/**
* @brief  ����λ�û���������ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
* @author ACTION
* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
*/
void SetPosLoopVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel)
{
	//��һ�����ݷ���SP����ڶ������ݷ�������ֵ
	uint32_t data[1][2]={
							0x00005053,0x00000000,		//SP
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = vel;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//�ȴ����ͳɹ�
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//������Ӧ�����쳣����
		
		}
	}
}

/**
* @brief  ���������ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
* @author ACTION
* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
*/
void SetJoggingVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel)
{
	//��һ�����ݷ���JV����ڶ������ݷ�������ֵ
	uint32_t data[1][2]={
							0x0000564A,0x00000000,		//JV
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = vel;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//�ȴ����ͳɹ�
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//������Ӧ�����쳣����
		
		}
	}
}

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
void SendPosCmd(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos)
{

	uint32_t data[1][2]={
							0x00000000,0x00000000,      //PA
						 };

	uint8_t mbox;
	CanTxMsg TxMessage;

	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	if(posMode==ABSOLUTE_MODE)
	{
		data[0][0]= 0x00004150;  //����
	}
	else if(posMode==RELATIVE_MODE)
	{
		data[0][0]= 0x00005250;   //���
	}

	data[0][1]= pos;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
	
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}

}

/**
* @brief  ��ʼ��һ���˶�����
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
* @note��
*/
void BeginMotion(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	//��һ�����ݷ���BG����
	uint32_t data[1][2]={
							0x40004742,0x00000000,    //BG
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//�ȴ����ͳɹ�
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//������Ӧ�����쳣����
		
		}
	}
}

/**********************************��ȡ��������������*************************************/

/**
* @brief  ��ȡ�����ѹ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
* @note��
*/
void ReadActualVoltage(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005155,0x00000000,      //UQ
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
 }

/**
* @brief  ��ȡ�������
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x80005149
*/
void ReadActualCurrent(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005149,0x00000000,      //IQ
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	} 
 }

/**
* @brief  ��ȡ���λ��
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00005850
*/
void ReadActualPos(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005850,0x00000000,      //PX
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			timeout=timeout;
		}
	} 
 }

/**
* @brief  ��ȡ����ٶ�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00005856
*/
void ReadActualVel(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40005856,0x00000000,      //VX
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			;
		}
	}
}

/**
* @brief  ��ȡ�������¶�
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00014954
*/
void ReadActualTemperature(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40014954,0x00000000,      //TI[1]
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}

/**
* @brief  ��ȡ�������������Ʊ�־λ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x0000434C
*/
void ReadCurrentLimitFlag(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000434C,0x00000000,      //LC
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}

/**
* @brief  ��ȡ�������ٶ�������������ٶ���ʵ���ٶȵ���
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��
*/
void ReadVelocityError(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004556,0x00000000,      //VE
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
	
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}

/**
* @brief  ��ȡ������������ٶ�����ֵ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x00025644
*/
void ReadCommandVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40025644,0x00000000,      //DV[2]
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
		
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}

/**
* @brief  ��ȡ������յ����ٶ�����
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x0000564A
*/
void ReadJoggingVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000564A,0x00000000,      //JV
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);        
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}

/**
* @brief  ��ȡ���ģʽ
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��
*/
void ReadUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004D55,0x00000000,      //UM
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}

/**
* @brief  ��ȡ������RM״̬
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��
*/
void ReadReferenceMode(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004D52,0x00000000,      //RM
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}

/**
* @brief  ��ȡ�������������
* @param  CANx����ʹ�õ�CANͨ�����
* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
* @author ACTION
 * @note�����ձ�ʶ��Ϊ��0x0000464D
*/
void ReadMotorFailure(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000464D,0x00000000,      //MF
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage); 
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			
		}
	}
}



