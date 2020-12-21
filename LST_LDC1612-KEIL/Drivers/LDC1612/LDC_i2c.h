/**
  ******************************************************************************
  * @file    LDC_i2c.h
  * @author  zafer sen
  * @version V1.0.0
  * @date    2018.08.29
  * @brief   This file create for XXX 		operation Functions prototypes 
  *          ,macros and definitions.
  
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LDC_I2C_H__
#define __LDC_I2C_H__
/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "stm32f0xx_hal.h"
#include <stdbool.h>

/* Exported macro ------------------------------------------------------------*/

#define proximityValue value

#define proximityValueKalman value

#define LED_GRN_GPIO_Port GPIOC
#define LED_GRN_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_5
#define LDC_OK    1
#define LDC_FAIL  0

/* Exported types ------------------------------------------------------------*/  

typedef enum
{
	I2C_RD = 0x01U,
	I2C_WRT = 0x00U
} flags_r_w;

struct LDC_I2C_Value_t
{
	uint8_t regAdress;
	struct
	{
		uint8_t msb;
		uint8_t lsb;
	} bytes;
};

struct i2c_msg
{
	flags_r_w flagsWR; /* Okuma yazma ve diger seçenekler read=1 write=0*/
	char *buf; /* tampon alanin baslangiç adresi */
	short len; /* tampon alanin uzunlugu */
};

struct i2c_rdwr_ioctl_data
{
	struct i2c_msg *iomsgs;
	uint32_t nmsgs; /* mesaj sayisi */
};
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern volatile uint32_t value;
extern _Bool ldcErrMSBStatus;
extern uint32_t firstCalData, runCalData;
extern int32_t Ndata;
extern _Bool ldcIniStatus;
extern uint8_t ldcInitRtryCnt;
extern uint8_t ldcStatus;
/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef LDC_Startup( void );
HAL_StatusTypeDef LDC_Config( void );
void LDC_Control( void );
bool I2C_ReadLDCWhoAmI( void );

#endif /* __LDC_I2C_H__ */

/***********END OF FILE*****************/
