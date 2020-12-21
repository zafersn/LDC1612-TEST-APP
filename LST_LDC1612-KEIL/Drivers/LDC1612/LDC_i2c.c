/**
  ******************************************************************************
  * @file    LDC1612_i2c.c
  * @author  ZAFER SEN
  * @version V1.0.0
  * @date    2018.08.29
  * @brief   This file create for 
  *
  * @verbatim	
  *			
  *		V1.0.0 : 	First version of library. 	
  *
  * @endverbatim
  * ===========================================================================
  *                       ##### How to use this driver #####
  * =========================================================================== 
  *	@attention 
  *	
  *	@todo
  *
  *		
  *	    
  *	
  */
  
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stm32f030x8.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_def.h>
#include <stm32f0xx_hal_i2c.h>
#include "LDC_i2c.h"
#include "LDC1612_driver.h"

extern I2C_HandleTypeDef hi2c1;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/	

#define I2C_TIMEOUT_TICK        100
/* Private typedef -----------------------------------------------------------*/	
/* Private variables ---------------------------------------------------------*/

//char *ad, *ag, *af;
volatile _Bool completionFlag = false;						///*
volatile _Bool completionFlagTX = false;					//*i2c callback flags
volatile _Bool completionFlagRX = false;					//*
volatile _Bool LDC_I2C_Error_Flag = false;				//**/



volatile _Bool TASK_50MS = false;
volatile uint16_t task_50ms_cnt = 0;
volatile _Bool TASK_1000MS = false;
volatile uint16_t task_1000ms_cnt = 0;

volatile uint32_t value; //okunan raw datalar 

uint8_t ldcStatus;					
_Bool ldcIniStatus = LDC_OK;
uint8_t ldcInitRtryCnt = 0;


//const static uint8_t sampleCount = 10;
//Startup LDC REG Values

const static uint16_t Startup_LDC_MUX_CONFIG =																				//ldc config ayarlarıdır. LDC1612_REG_MUX_CONFIG yani 0x1B adresine yazılacak config ayarı. 
	( AUTOSCAN_EN | DEGLITCH_3N3MHZ | MUX_CONFIG_RESERVED );														//DEGLITCH_3N3MHZ ise hesaplama şeklimiz 1/(2*pi*sqrt(17.7*10^-6*330*10'-12)) = 2082459.3302 ~ 2mhz en yakın olarak 3.3mhz seçildi.


const static uint16_t Startup_LDC_CONFIG_SLEEP_DISABLE = ( ACTIVE_CHAN_1 | SLEEP_MODE_DISABLE   //sleep moddan çıkmak için.
	| RP_OVERRIDE_EN | SENSOR_ACTIVATE_SEL | AUTO_AMP_DIS | REF_CLK_SRC | CONFIG_RESERVED | INTB_DIS
	| RESERVED_REG_CONFIG );

const static uint16_t Startup_LDC_CONFIG_SLEEP_EN = ( ACTIVE_CHAN_1 | SLEEP_MODE_EN | RP_OVERRIDE_EN //sleep moda girip config ayarları yapmak için
	| SENSOR_ACTIVATE_SEL | AUTO_AMP_DIS | REF_CLK_SRC | CONFIG_RESERVED |INTB_DIS| HIGH_CURRENT_DRV_EN
	| RESERVED_REG_CONFIG );

//static const uint16_t Sleep_LDC_CONFIG = SLEEP_MODE_EN;

static const uint16_t Startup_LDC_ERROR_CONFIG = 0x0000;		
_Bool ldcErrMSBStatus = false;
uint16_t Current_LDC_Config; // test için koydum register 'a yazılacak hex değerlerini kontrol etmek amacıyla

/***** < config settings >****/

// Sensor CH1
//static const uint16_t Startup_LDC_CLOCK_DIVIDERS_CH1 = 0x100D;
static const uint16_t Startup_LDC_CLOCK_DIVIDERS_CH1 = 0x1002;				
//static const uint16_t Startup_LDC_SETTLECOUNT_CH1 = 1024;
//static const uint16_t Startup_LDC_SETTLECOUNT_CH1 = 0x000B;
static const uint16_t Startup_LDC_SETTLECOUNT_CH1 = 0x000A;
//static const uint16_t Startup_LDC_RCOUNT_CH1 = 0x1388;
//static const uint16_t Startup_LDC_RCOUNT_CH1 = 0xffff;
static const uint16_t Startup_LDC_RCOUNT_CH1 = 0x04D6;
static const uint16_t Startup_LDC_OFFSET_CH1 = 0x0000;
//static const uint16_t Startup_LDC_DRIVE_CURRENT_CH1 = 0xC600;
static const uint16_t Startup_LDC_DRIVE_CURRENT_CH1 = 0x5800; // Rp Ranges For IDRIVE1	{INDEX : 11 , b01011 }

/***** </ config settings >****/

/* Private function prototypes -----------------------------------------------*/

/***** < function declaration >****/

HAL_StatusTypeDef i2c_master_write( uint8_t *data, uint16_t size );  			//i2c ile ldc ye data yazdığımız func.

HAL_StatusTypeDef i2c_master_read( I2C_HandleTypeDef *obj, uint8_t dev_address,
	uint8_t *data, uint16_t size );																					//i2c ile ldc den data okuduğumuz func.
	
uint32_t LDC_Read_Data( void );						//I2C ile sensör datasını (msb ,lsb) okudumuz ve birleştirdiğimiz func.
	
void LDC_Wakeup( void );									//ldc 'yi config ayarları ile birlikte sleep moddan çıkartır.

void LDC_Sleep( void );									//ldc 'yi config ayarları ile birlikte sleep moddan sokar.

void LDC_Config_All( void );						//ldc için başlangıçta(sleep modda) yapılması gereken bütün config ayarlarının yapıldığı func.

_Bool LDC_STATUS_UNREADCONV1( void );				//LDC 'DE   okunmamış bir dönüşüm var mı diye kontrol ediyoruz.

HAL_StatusTypeDef LDC_Abort_Transfers( void );	//Interrupt ile bir master I2C IT işlem iletişimini iptal eder.

_Bool isSleepModeTEST( void );							//test için yazıldı. okunan register adreslerindeki dataları görmek için.

_Bool isModeTest( void );										//test için yazıldı. okunan register adreslerindeki dataları görmek için.

/***** </ function declaration >****/


/* Private functions ---------------------------------------------------------*/ 

HAL_StatusTypeDef LDC_Startup( void )
    {
HAL_StatusTypeDef ret = HAL_ERROR;
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET );	//i2c +5v kanalı açtık
_Bool isOK = false;
if ( HAL_I2C_GetState(( &hi2c1 ) ) == HAL_I2C_STATE_READY )  
{
	ret = HAL_OK;
}
else
{
	ldcIniStatus = LDC_FAIL;
	ret = HAL_ERROR;
}
if ( ret == HAL_OK )
{
	isOK = I2C_ReadLDCWhoAmI( );// kullandığımız cihazın device id sini okuduk ve doğruladık.
	if ( isOK )
	{
		ret = LDC_Config( );
		HAL_Delay(20 );
		if ( ret == HAL_OK )
		{
			LDC_Control( );
		}
	}
	else
	{
			ret = HAL_ERROR;
			ldcIniStatus = LDC_FAIL;
	}
}
return ret;
}

HAL_StatusTypeDef LDC_Config( void )
{
	
	//sırayla config ayarlarını yapıyoruz
	Current_LDC_Config = Startup_LDC_CONFIG_SLEEP_EN;
	LDC_Config_All( );
	LDC_Wakeup( );                              //sleep moddan çıkart ve ölçümlere başla
	isSleepModeTEST( );															//test
	ldcIniStatus = LDC_OK;
	ldcInitRtryCnt = 0;

	return HAL_OK;
}
/**
 * @brief Main loop function
 * @param None.
 * @return None
 */
void LDC_Control( void )
{
	while ( 1 )
	{

		// Get the data from the sensors
		if ( LDC_STATUS_UNREADCONV1() ) 		//okunmamış değer varmı bakıyoruz ? varsa okuyoruz.
		{
			uint32_t raw_value = LDC_Read_Data( );		// dataları okuduğumuz func.
			if ( raw_value != value )
			{
					value = raw_value;			
					isModeTest();
					isSleepModeTEST( );
			}
			//handleCommunication( );		//UART communication func.
		}						
		if ( TASK_50MS )
		{
			TASK_50MS = false;

		}
		if ( TASK_1000MS )
		{
			TASK_1000MS = false;			
			//Manage periodic LDC/SDK tasks			
		}
	}
}

/**
 * @brief Hal library I2C master receive completed callback function
 * @param None.
 * @return Read status
 */
void HAL_I2C_MasterRxCpltCallback( I2C_HandleTypeDef *hi2c )
{
	if ( hi2c->Instance == I2C1 )
	{
		completionFlag = completionFlagRX = true;
	}
}

/**
 * @brief Hal library I2C master transmit completed callback function
 * @param None.
 * @return Read status
 */
void HAL_I2C_MasterTxCpltCallback( I2C_HandleTypeDef *hi2c )
{
	if ( hi2c->Instance == I2C1 )
	{
		completionFlag = completionFlagTX = true;
	}
}

void HAL_I2C_ErrorCallback( I2C_HandleTypeDef *hi2c )
{
	LDC_I2C_Error_Flag = true;
	completionFlag = false;
}
			/***** < Bu kısm test edilmedi >****/
HAL_StatusTypeDef LDC_Abort_Transfers( void )
{
	return HAL_I2C_Master_Abort_IT(&hi2c1, LDC1612_MIN_I2CADDR );
}
	
HAL_StatusTypeDef LDC_I2C_DeInit( void )
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	LDC_Abort_Transfers( );
	HAL_Delay(20 );
	ret = HAL_I2C_DeInit(&( hi2c1 ) );
	return ret;

}

HAL_StatusTypeDef LDC_I2C_ReInit( void )
{
		HAL_Delay(20 );
		ldcInitRtryCnt++;
		return HAL_I2C_Init(&hi2c1 );
}
			/***** </ ^|^ Bu kısm test edilmedi >****/ 
		/***** < i2c haberleşmede yazma func >****/
HAL_StatusTypeDef i2c_master_write( uint8_t *data, uint16_t size )
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint32_t tickstart = HAL_GetTick( );
	uint32_t delta = 0;
	I2C_HandleTypeDef *obj = &hi2c1;
	uint8_t dev_address = LDC1612_MIN_I2CADDR;
	do
	{
		if ( HAL_I2C_Master_Transmit_IT(( obj ), dev_address, data, size )
		== HAL_OK )
		{
			ret = HAL_OK;
			// wait for transfer completion
			while ( ( HAL_I2C_GetState(( obj ) ) != HAL_I2C_STATE_READY )
			&& ( ret == HAL_OK ) )
			{
				delta = ( HAL_GetTick( ) - tickstart );
				if ( delta > I2C_TIMEOUT_TICK )
				{
					ret = HAL_TIMEOUT;
				}
				else if ( HAL_I2C_GetError(obj ) != HAL_I2C_ERROR_NONE )
				{
					ret = HAL_ERROR;
				}
			}
		}
/* When Acknowledge failure occurs (Slave don't acknowledge it's address)
 Master restarts communication */
	}
	while ( HAL_I2C_GetError(( obj ) ) == HAL_I2C_ERROR_AF
		&& delta < I2C_TIMEOUT_TICK );

return ret;
}			
		/***** </ i2c haberleşmede yazma func >****/		
		/***** < i2c haberleşmede okuma func >****/
HAL_StatusTypeDef i2c_master_read( I2C_HandleTypeDef *obj, uint8_t dev_address,
	uint8_t *data, uint16_t size )
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint32_t tickstart = HAL_GetTick( );
	uint32_t delta = 0;

	do
	{
		if ( HAL_I2C_Master_Receive_IT(( obj ), dev_address, data, size )
			== HAL_OK )
		{
			ret = HAL_OK;
			// wait for transfer completion
			while ( ( HAL_I2C_GetState(obj ) != HAL_I2C_STATE_READY )
				&& ( ret == HAL_OK ) )
			{
				delta = ( HAL_GetTick( ) - tickstart );
				if ( delta > I2C_TIMEOUT_TICK )
				{
					ret = HAL_TIMEOUT;
				}
				else if ( HAL_I2C_GetError(( obj ) ) != HAL_I2C_ERROR_NONE )
				{
					ret = HAL_ERROR;
				}
			}
		}
		/* When Acknowledge failure occurs (Slave don't acknowledge it's address)
		 Master restarts communication */
	}
	while ( HAL_I2C_GetError(( obj ) ) == HAL_I2C_ERROR_AF
		&& delta < I2C_TIMEOUT_TICK );
	return ret;
}

/***** </ i2c haberleşmede okuma func >****/
		/***** < ldc 'den  veri okumak için i2c frame yapısa uygun okuma func. >****/
HAL_StatusTypeDef i2c_LDC_Read_Data( struct i2c_rdwr_ioctl_data *msgs )
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint32_t delta = 0;
	uint32_t tickstart = HAL_GetTick( );

	for ( int i = 0; i < msgs->nmsgs; i++ )
	{
		while ( ( HAL_I2C_GetState(( &hi2c1 ) ) != HAL_I2C_STATE_READY ) )
		{
			ret = HAL_OK;
			delta = ( HAL_GetTick( ) - tickstart );
			if ( delta > I2C_TIMEOUT_TICK )
			{
				ret = HAL_TIMEOUT;
				break;
			}
			else if ( HAL_I2C_GetError(&hi2c1 ) != HAL_I2C_ERROR_NONE )
			{
				ret = HAL_ERROR;
				break;
			}
		}
		if ( ret != HAL_OK )
		{
			ret = LDC_I2C_DeInit( );
			if ( ret == HAL_OK )
				ret = LDC_I2C_ReInit( );
		}
		if ( ret == HAL_OK && msgs->iomsgs[ i ].flagsWR == 0 )
		{
			ret = i2c_master_write(( uint8_t* ) ( msgs->iomsgs[ i ].buf ),
			msgs->iomsgs[ i ].len );
		}
		if ( ret == HAL_OK && msgs->iomsgs[ i ].flagsWR == 1 )
		{
			ret = i2c_master_read(
						&hi2c1,
						LDC1612_MIN_I2CADDR | 0x01,
						( uint8_t* ) ( msgs->iomsgs[ i ].buf ),
						msgs->iomsgs[ i ].len );
		}
	}
	return ret;
}
		/***** </ ldc 'den  veri okumak için i2c frame yapısa uygun okuma func. >****/
				/***** < ldc 'de DEVICE_ID registerini okudğumuz yer. >****/

_Bool I2C_ReadLDCWhoAmI( void )
{
	_Bool find_device = false;
	struct LDC_I2C_Value_t message;
	struct i2c_rdwr_ioctl_data msgs;
	struct i2c_msg iomsgs[ 2 ];

	HAL_StatusTypeDef ret;
	message.regAdress = LDC1612_REG_DEVID;
	iomsgs[ 0 ].flagsWR = I2C_WRT; /* Okuma yazma ve diger seçenekler read=1 write=0*/

	iomsgs[ 0 ].buf = ( char* ) ( &message );
	iomsgs[ 0 ].len = 1;

	/*okuma islemi*/
	iomsgs[ 1 ].flagsWR = I2C_RD;
	iomsgs[ 1 ].buf = ( char* ) ( &message ) + ( 1 );
	iomsgs[ 1 ].len = 2;

	msgs.iomsgs = iomsgs;
	msgs.nmsgs = 2;

	if ( HAL_I2C_GetState(( &hi2c1 ) ) == HAL_I2C_STATE_READY )
	{
		ret = HAL_OK;
	}
	else
	{
		ret = HAL_ERROR;
	}
	if ( ( hi2c1.State == HAL_I2C_STATE_READY ) && ( ret == HAL_OK ) )
	{
		find_device = true;
	}
	if ( find_device )
	{
		if ( ( i2c_LDC_Read_Data(&msgs ) == HAL_OK ) && completionFlagRX )
		{
			completionFlagRX = false;
			if ((( message.bytes.msb << 8 ) | message.bytes.lsb)  == IamLDC1612 )
			{
				return true;
			}
			return false;
		}
	}
	return false;
}
				/***** </ ldc 'de DEVICE_ID registerini okudğumuz yer. >****/

				/***** < ldc config >****/		
void LDC_Config_All( void )
{
	struct LDC_I2C_Value_t message;
	message.regAdress = LDC1612_REG_RESET_DEVICE;
	message.bytes.msb = ( RESET_DEV >> 8 );
	message.bytes.lsb = RESET_DEV & 0xff;

	// software reset
	i2c_master_write(( uint8_t * ) &message, sizeof(message) );// resetledik ve sleep modda başladı
	HAL_Delay(10);
			
	LDC_Sleep( );

	message.regAdress = LDC1612_REG_MUX_CONFIG;
	message.bytes.msb = ( Startup_LDC_MUX_CONFIG >> 8 );
	message.bytes.lsb = Startup_LDC_MUX_CONFIG & 0xff;
	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );

	message.regAdress = LDC1612_REG_ERROR_CONFIG;
	message.bytes.msb = ( Startup_LDC_ERROR_CONFIG >> 8 );
	message.bytes.lsb = Startup_LDC_ERROR_CONFIG & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );

	//CH1
	message.regAdress = LDC1612_REG_CLOCK_DIVIDERS_CH1;
	message.bytes.msb = ( Startup_LDC_CLOCK_DIVIDERS_CH1 >> 8 );
	message.bytes.lsb = Startup_LDC_CLOCK_DIVIDERS_CH1 & 0xff;

	i2c_master_write(( uint8_t * ) &message,sizeof(message)  );

	message.regAdress = LDC1612_REG_SETTLE_COUNT_CH1;
	message.bytes.msb = ( Startup_LDC_SETTLECOUNT_CH1 >> 8 );
	message.bytes.lsb = Startup_LDC_SETTLECOUNT_CH1 & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );

	message.regAdress = LDC1612_REG_REF_COUNT_CH1;
	message.bytes.msb = ( Startup_LDC_RCOUNT_CH1 >> 8 );
	message.bytes.lsb = Startup_LDC_RCOUNT_CH1 & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );

	message.regAdress = LDC1612_REG_OFFSET_CH1;
	message.bytes.msb = ( Startup_LDC_OFFSET_CH1 >> 8 );
	message.bytes.lsb = Startup_LDC_OFFSET_CH1 & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );

	message.regAdress = LDC1612_REG_DRIVE_CURRENT_CH1;
	message.bytes.msb = ( Startup_LDC_DRIVE_CURRENT_CH1 >> 8 );
	message.bytes.lsb = Startup_LDC_DRIVE_CURRENT_CH1 & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );

	message.regAdress = LDC1612_REG_CONFIG;
	message.bytes.msb = ( Startup_LDC_CONFIG_SLEEP_DISABLE >> 8 );
	message.bytes.lsb = Startup_LDC_CONFIG_SLEEP_DISABLE & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );
}
/***** </ ldc config >****/
/***** < ldc sleep moda geç.. >****/		
void LDC_Sleep( void )
{
	struct LDC_I2C_Value_t message;
	Current_LDC_Config |= Startup_LDC_CONFIG_SLEEP_EN;
	message.regAdress = LDC1612_REG_CONFIG;
	message.bytes.msb = ( Startup_LDC_CONFIG_SLEEP_EN >> 8 );
	message.bytes.lsb = Startup_LDC_CONFIG_SLEEP_EN & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );
}
		/***** </ ldc sleep moda geç.. >****/		
		/***** < ldc sleep modadan çık.. >****/		
void LDC_Wakeup( void )
{
	struct LDC_I2C_Value_t message;
	Current_LDC_Config &= ~( Startup_LDC_CONFIG_SLEEP_DISABLE );
	message.regAdress = LDC1612_REG_CONFIG;
	message.bytes.msb = ( Startup_LDC_CONFIG_SLEEP_DISABLE >> 8 );
	message.bytes.lsb = Startup_LDC_CONFIG_SLEEP_DISABLE & 0xff;

	i2c_master_write(( uint8_t * ) &message, sizeof(message)  );
}
		/***** </ ldc sleep modadan çık.. >****/				
		/***** < testtttttt >****/		
_Bool isSleepModeTEST( void )
{
	struct LDC_I2C_Value_t message;
	struct i2c_rdwr_ioctl_data msgs;
	struct i2c_msg iomsgs[ 2 ];

	message.regAdress = LDC1612_REG_CONFIG;

	iomsgs[ 0 ].flagsWR = I2C_WRT; /* Okuma yazma ve diger seçenekler read=1 write=0*/
	//  iomsgs[0].memAddr=LDC1612_REG_DEVID;
	//  as[0]=LDC1612_REG_DEVID;

	iomsgs[ 0 ].buf = ( char* ) ( &message );
	iomsgs[ 0 ].len = 1;

	/*okuma islemi*/
	iomsgs[ 1 ].flagsWR = I2C_RD;
	iomsgs[ 1 ].buf = ( char* ) ( &message ) + ( 1 );
	iomsgs[ 1 ].len = 2;

	msgs.iomsgs = iomsgs;
	msgs.nmsgs = 2;

	if ( ( i2c_LDC_Read_Data(&msgs ) == HAL_OK ) && completionFlagRX )
	{
		if ( ( ( message.bytes.msb << 8 ) | message.bytes.lsb) == IamLDC1612 )
		{
			return true;
		}
	return false;
	}
	return false;
}
_Bool isModeTest( void )
{
	struct LDC_I2C_Value_t message;
	struct i2c_rdwr_ioctl_data msgs;
	struct i2c_msg iomsgs[ 2 ];
	message.regAdress = LDC1612_REG_DRIVE_CURRENT_CH1;
	iomsgs[ 0 ].flagsWR = I2C_WRT; /* Okuma yazma ve diger seçenekler read=1 write=0*/
	//  iomsgs[0].memAddr=LDC1612_REG_DEVID;
	//  as[0]=LDC1612_REG_DEVID;
	iomsgs[ 0 ].buf = ( char* ) ( &message );
	iomsgs[ 0 ].len = 1;
	/*okuma islemi*/
	iomsgs[ 1 ].flagsWR = I2C_RD;
	iomsgs[ 1 ].buf = ( char* ) ( &message ) + ( 1 );
	iomsgs[ 1 ].len = 2;
	msgs.iomsgs = iomsgs;
	msgs.nmsgs = 2;
	if ( ( i2c_LDC_Read_Data(&msgs ) == HAL_OK ) && completionFlagRX )
	{
		if ( ( ( message.bytes.msb << 8 ) | message.bytes.lsb) == IamLDC1612 )
		{
			return true;
		}
		return false;
	}
	return false;
}
/***** </ testtttttt >****/	
/***** < ldc 'den value değerlerini okuyup birleştirdiğimiz kısım. >****/
uint32_t LDC_Read_Data( void )
{
	struct LDC_I2C_Value_t ldc_value;
	struct i2c_rdwr_ioctl_data msgs;
	struct i2c_msg iomsgs[ 2 ];
//	HAL_StatusTypeDef ret;
        uint16_t dataMSB,dataLSB;
	ldc_value.regAdress = LDC1612_REG_DATA_MSB_CH1;
	iomsgs[ 0 ].flagsWR = I2C_WRT; /* Okuma yazma ve diger seçenekler read=1 write=0*/
	iomsgs[ 0 ].buf = ( char* ) ( &ldc_value );
	iomsgs[ 0 ].len = 1;
	/*okuma islemi*/
	iomsgs[ 1 ].flagsWR = I2C_RD;
	iomsgs[ 1 ].buf = ( char* ) ( &ldc_value ) + ( 1 );
	iomsgs[ 1 ].len = 2;
	//isSleepMode();
	msgs.iomsgs = iomsgs;
	msgs.nmsgs = 2;
	if ( ( i2c_LDC_Read_Data(&msgs ) == HAL_OK ) && completionFlagRX )
	{
		completionFlagRX = false;
		if ( ldc_value.bytes.msb & 0xF0 )
		{
			//Error detected.
			ldcStatus |= 1UL << 7;
			ldcErrMSBStatus = true;
			return 0;
		}
		ldcErrMSBStatus = false;
		dataMSB =  ( ( ldc_value.bytes.msb & 0x0F ) << 8 )
			| ldc_value.bytes.lsb ;
	}
	ldc_value.regAdress = LDC1612_REG_DATA_LSB_CH1;
	iomsgs[ 0 ].flagsWR = I2C_WRT; /* Okuma yazma ve diger seçenekler read=1 write=0*/
	iomsgs[ 0 ].buf = ( char* ) ( &ldc_value );
	iomsgs[ 0 ].len = 1;
	/*okuma islemi*/
	iomsgs[ 1 ].flagsWR = I2C_RD;
	iomsgs[ 1 ].buf = ( char* ) ( &ldc_value ) + ( 1 );
	iomsgs[ 1 ].len = 2;
	msgs.iomsgs = iomsgs;
	msgs.nmsgs = 2;
	if ( ( i2c_LDC_Read_Data(&msgs ) == HAL_OK ) && completionFlagRX )
	{
		completionFlagRX = false;
		dataLSB = ( ldc_value.bytes.msb << 8 )
		| ldc_value.bytes.lsb ;
	}
		//http://www.ti.com/lit/ds/symlink/ldc1612.pdf#page=10&zoom=100,0,856 sayf 38
	return dataMSB * 65536 + dataLSB;
}		
	/***** </ ldc 'den value değerlerini okuyup birleştirdiğimiz kısım. >****/	
	/***** <LDC 'DE   okunmamış bir dönüşüm var mı diye kontrol ediyoruz.>****/
_Bool LDC_STATUS_UNREADCONV1( void )
{
	struct LDC_I2C_Value_t ldc_value;
	struct i2c_rdwr_ioctl_data msgs;
	struct i2c_msg iomsgs[ 2 ];
	ldc_value.regAdress = LDC1612_REG_STATUS;
	iomsgs[ 0 ].flagsWR = I2C_WRT; /* Okuma yazma ve diger seçenekler read=1 write=0*/
	iomsgs[ 0 ].buf = ( char* ) ( &ldc_value );
	iomsgs[ 0 ].len = 1;

	/*okuma islemi*/
	iomsgs[ 1 ].flagsWR = I2C_RD;
	iomsgs[ 1 ].buf = ( char* ) ( &ldc_value ) + ( 1 );
	iomsgs[ 1 ].len = 2;
	//isSleepMode();
	msgs.iomsgs = iomsgs;
	msgs.nmsgs = 2;
	if ( ( i2c_LDC_Read_Data(&msgs ) == HAL_OK ) && completionFlagRX )
	{
		completionFlagRX = false;
		if ( ldc_value.bytes.lsb & LDC1612_BIT_UNREADCONV1 )
		{
			//new data detected.
			ldcStatus |= 1UL << 2;
			return true;
		}
		ldcStatus &= ~( 1UL << 2 );
		return false;
	}
	ldcStatus &= ~( 1UL << 2 );
	return false;
}

	/***** </LDC 'DE   okunmamış bir dönüşüm var mı diye kontrol ediyoruz.>****/
	//****************** END OF THE PAGE **************************/
