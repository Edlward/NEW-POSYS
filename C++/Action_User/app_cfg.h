/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               app_cfg.h
** Descriptions:            ucosii configuration
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-9
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__
#include  <os_cpu.h>					  

/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              TASKS NAMES
*********************************************************************************************************
*/
extern  void  App_Task(void);

static  void  App_TaskStart(void); 
static  void ConfigTask(void);
static  void UpdateTask(void);


/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_TASK_START_PRIO                               10u
#define  Config_TASK_START_PRIO                            11u
#define  UPDATE_START_PRIO                                 12u

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/
#define  APP_TASK_START_STK_SIZE                          256u
#define  Config_TASK_START_STK_SIZE                       256u
#define  TASK1_START_STK_SIZE                             8192u


/*
*********************************************************************************************************
*                                            TASK STACK
*                             
*********************************************************************************************************
*/
static  OS_STK  App_TaskStartStk[APP_TASK_START_STK_SIZE];
static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];
static  OS_STK  Task1Stk[TASK1_START_STK_SIZE];


/*
*********************************************************************************************************
*                                                  LIB
*********************************************************************************************************
*/



#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

