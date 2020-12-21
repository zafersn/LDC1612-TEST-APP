/**
  ******************************************************************************
  * @file    UART_Communication.h
  * @author  Ahmet Açikgöz
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
#ifndef __UART_COMMUNICATION_H_
#define __UART_COMMUNICATION_H_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#define UART_MAX_PACKAGE_LENGHT					64
#define UART_RAW_BUFFER_SIZE					UART_MAX_PACKAGE_LENGHT

#define UART_END_OFF_TIME						5
#define LED_GRN_GPIO_Port GPIOC
#define LED_GRN_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_5
#define UC_UART1								0

#define UC_TOTAL_UART_COUNT						1

/* Exported types ------------------------------------------------------------*/  

/**
 * @brief UART object handler
 */
typedef struct{

	struct													/*!< (1 / 0)*/
	{
		unsigned short New_Data_Start:1;					/*!< New message start flag */
		unsigned short New_Data_Package_Accepted:1;			/*!< New message packet accepted flag */
		unsigned short Transmit_Active:1;					/*!<  */
		unsigned short Tx_Completed:1;	
		
	}Flags; 												/** OBject global flags*/

	unsigned char Raw_Buffer[UART_RAW_BUFFER_SIZE];			/** UART receive Message Buffer */
	unsigned char Raw_Buffer_Cnt;							/** UART receive Message Buffer size register  */

	unsigned char Receive_Data;								/** UART Single Receive data for interrupt */

	unsigned short Data_End_Off_Time_Counter;				/** End of time counter*/

	unsigned char Transmit_Data_Buffer[UART_RAW_BUFFER_SIZE];
	unsigned char Transmit_Data_Cnt;
	unsigned char Transmit_Data_Total_Cnt;
	
}UART_Object_Typedef;

/**
 * @brief UART Driver object handler
 */
typedef struct
{
	struct														/*!< (1 / 0)*/
	{
		unsigned short Device_Config:1;     					/*!< Device configuration result (Configured / not Configured)*/
		unsigned short Device_Startup:1;     					/*!< Device startup information  (Startup / not startup)*/
		

	}Flags; 													/** OBject global flags*/
		
	UART_Object_Typedef		UART_DO[UC_TOTAL_UART_COUNT];		/** UART object handler */

}UC_Driver_Object_Handler_Type;

	
	extern UC_Driver_Object_Handler_Type	UC_DOH;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

	extern void UC_Startup(void);
	extern void UC_Transmit_Message(char*  message);
	extern void UC_Check_Communication(void);
	extern unsigned char UC_Check_Receive(char*  message);
	extern void UC_IRQ_Virtual_RX(unsigned char UART_No,unsigned char UART_RXN);
	extern void UC_IRQ_Virtual_Systick(void);

#endif 

/*** (C) COPYRIGHT KRNK AR-GE Müh. ve Dan. Hizm. LTD. STI. *****END OF FILE****/

