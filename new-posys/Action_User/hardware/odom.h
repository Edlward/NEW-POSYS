#ifndef __ODOM_H
#define __ODOM_H

typedef struct{
	int odom1;
	int odom2;
}Odom_t;

#define ODOM_INVALID_PARAM_ERROR					0
#define ODOM_NOT_READY_ERROR							1
#define ODOM_READ_TIMEOUT_ERROR1					2
#define ODOM_READ_TIMEOUT_ERROR2					3
#define ODOM_READ_OK											4
#define ODOM_INCREMENTAL_MODE							5

void ODOM_Setup(void);

void ODOM_Configure(void);

static int ODOM_ReadRawData(Odom_t* rawData);

int ODOM_Read(Odom_t* odomData, int mode);

int ODOM_Check(void);

int ODOM_FatigureTest(void);



#endif

