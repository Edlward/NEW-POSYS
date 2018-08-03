#ifndef __ODOM_H
#define __ODOM_H
	
//#define TLE5012_USED
/* TLE5012 GPIO PIN define -----------------------------------------------------------*/
#define      TLE5012_SDN_APBxClock_FUN                    RCC_APB2PeriphClockCmd
#define      TLE5012_SDN_CLK                              RCC_APB2Periph_GPIOB    
#define      TLE5012_SDN_PORT                             GPIOB
#define      TLE5012_SDN_PIN                              GPIO_Pin_1

#define      TLE5012_IRQ_APBxClock_FUN                    RCC_APB2PeriphClockCmd
#define      TLE5012_IRQ_CLK                              RCC_APB2Periph_GPIOB   
#define      TLE5012_IRQ_PORT                             GPIOB   
#define      TLE5012_IRQ_PIN                              GPIO_Pin_0

/* Exported define -----------------------------------------------------------*/
// TLE5012 工作状态
#define        IN_IDEL                  0
#define        IN_RCV                   1
#define        IN_SEND                  2

/* Exported variables --------------------------------------------------------*/
                       

/* Exported functions ------------------------------------------------------- */
void      TLE5012Init(SPI_TypeDef* SPIx);
void      TLE5012Process(SPI_TypeDef* SPIx);
uint16_t 	TLE5012ReadAbsPos_A(void);
uint16_t 	TLE5012ReadAbsPos_B(void);
void 			TurnOnTLE5012(void);
void 			TurnOffTLE5012(void);
uint16_t 	TLE5012_RW_SPIx(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t  *p_u16StartAddr, uint8_t u8DataSize, uint8_t u8Operate);
uint16_t	TLE5012ReadReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data);
uint16_t	TLE5012WriteReg(SPI_TypeDef* SPIx, uint16_t u16Command, uint16_t* pu16Data);
uint16_t SPI_ReadAS5045(uint8_t num);
/* Exported functions ------------------------------------------------------- */

/**
 * @defgroup TLE5012_define_Constant TLE5012寄存器 
 */
 
#define   WRITE               0						//SPI Write Operate
#define   READ                1         	//SPI Read Operate

#define   WR_REG              0x0000      /* Command word 最高位为 0 写命令 */
#define   RD_REG              0x8000			/* Command word 最高位为 1 读命令 */

#define		LockValue_LADDR					0x0000	/* Command word lock 4bit addresses 0x00:0x04 */
#define		LockValue_HADDR					0x5000	/* Command word lock 4bit addresses 0x05:0x11 */

#define   UPD_CMD              0x0400      /* Command word the 10th bit is 1, Access to values in update buffer */

#define READ_STAT_VALUE			0x8001
#define READ_ACSTAT_VALUE		0x8011
#define READ_ANGLE_VALUE		0x8021
#define READ_SPEED_VALUE		0x8031			//0x8031

#define WRITE_MOD1_VALUE		0x5061							//0_1010_0_000110_0001
#define MOD1_VALUE	0x0001
#define WRITE_MOD2_VALUE		0x5081							//0_1010_0_001000_0001
#define MOD2_VALUE	0x0809
#define WRITE_MOD3_VALUE		0x5091							//0_1010_0_001001_0001
#define MOD3_VALUE	0x0000
#define WRITE_MOD4_VALUE		0x50E1							//0_1010_0_001110_0001
#define MOD4_VALUE	0x0080											//12Bit 4096 绝对计数disable 0x0080; 绝对计数enable 0x0000

#define WRITE_IFAB_VALUE		0x50B1
#define IFAB_VALUE 					0x000D

/******* TLE5012寄存器地址 *******************************
STAT 			STATus register 00H
ACSTAT 		ACtivation STATus register 01H
AVAL 			Angle VALue register 02H
ASPD 			Angle SPeeD register 03H
AREV 			Angle REVolution register 04H
FSYNC 		Frame SYNChronization register 05H
MOD_1 		Interface MODe1 register 06H
SIL 			SIL register 07H
MOD_2 		Interface MODe2 register 08H
MOD_3 		Interface MODe3 register 09H
OFFX 			OFFset X 0AH
OFFY 			OFFset Y 0BH
SYNCH 		SYNCHronicity 0CH
IFAB 			IFAB register 0DH
MOD_4 		Interface MODe4 register 0EH
TCO_Y 		Temperature COefficient register 0FH
ADC_X 		ADC X-raw value 10H
ADC_Y 		ADC Y-raw value 11H
IIF_CNT 	IIF CouNTer value 20H
*******************************************************/
#define		STAT_ADDR 									0x00
#define		ACSTAT_ADDR 								0x01
#define		AVAL_ADDR 									0x02
#define		ASPD_ADDR 									0x03
#define		AREV_ADDR 									0x04

#define		FSYNC_ADDR 								0x05
#define		MOD_1_ADDR 										0x06
#define		SIL_ADDR 									0x07
#define		MOD_2_ADDR 										0x08
#define		MOD_3_ADDR 										0x09
#define		OFFX_ADDR 								0x0A
#define		OFFY_ADDR 								0x0B
#define		SYNCH_ADDR 								0x0C
#define		IFAB_ADDR 								0x0D
#define		MOD_4_ADDR 										0x0E

#define		TCO_Y_ADDR 								0x0F
#define		ADC_X_ADDR 										0x10
#define		ADC_Y_ADDR 										0x11
#define		IIF_CNT_ADDR 							0x20



#endif

