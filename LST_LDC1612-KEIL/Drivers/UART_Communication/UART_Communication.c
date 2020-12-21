/**
  ******************************************************************************
  * @file    UART_Communication.c
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

/** Device Peripheral Access Layer Header File. */
#include "stm32f0xx_hal.h"

#include "UART_Communication.h"
//#include "BSL.h"
/** Standard C library */
#include <stdlib.h>
#include <string.h>
	extern UART_HandleTypeDef huart1;

/* Private typedef -----------------------------------------------------------*/	

	/** UART Communication driver object */
	UC_Driver_Object_Handler_Type	UC_DOH;
	
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/	

#define HAL_UART1_OBJECT						huart1

#define UC_EXOR_FIXED_KEY						0x1C

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/ 

void UC_Startup(void)
{
	/** Clean driver object*/
	memset(&UC_DOH,0,sizeof(UC_DOH));
	/** Set UART receive interrupt */
	HAL_UART_Receive_IT(&HAL_UART1_OBJECT,&UC_DOH.UART_DO[UC_UART1].Receive_Data,1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{	
	UC_DOH.UART_DO[UC_UART1].Flags.Transmit_Active = 0;
}

/**
 * @brief UART error check function
 * @param usart_no : UART no
 * @retval None.
 */
void UC_Control_USART_Error()
{
	unsigned char data;

	if(__HAL_UART_GET_FLAG(&HAL_UART1_OBJECT,UART_FLAG_ORE))							/** Check UART error flag */
	{		
		HAL_UART_Receive(&HAL_UART1_OBJECT,&data,1,10);								/** Receive 1 byte */		
		HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0);
		HAL_UART_Receive_IT(&HAL_UART1_OBJECT,&UC_DOH.UART_DO[UC_UART1].Receive_Data,1);	/** Set UART receive interrupt */
	}	
}

/**
 * @brief LRC calculation function
 * @param pBuffer : Data buffer 
 * @param length : buffer length
 * @retval LRC value.
 */
unsigned char UC_Calculate_LRC(	unsigned char* pBuffer, 
								unsigned char length)
{
	unsigned char loop_cnt;								/** loop counter register */							
	unsigned char lrc = 13;								/** LRC Calculation registe ,Start value 13*/
	
	/** All buffer add to lrc register */
	for(loop_cnt = 0; loop_cnt < length; loop_cnt++)
		lrc += pBuffer[loop_cnt];
	
	/** return the lrc value*/
	return lrc;
}

/**
 * @brief UART message transmit function non polling mode 
 * @param message : UART message object @arg CPK_Serial_Message_Type
 * @retval None.
 * @note timing detail
 *		function minimum time ==> 1.2us
 *		function maximum time ==> 9.5us (for the 16 byte data)
 */
unsigned char BSL_Get_Random_Number(void)
{
	srand(HAL_GetTick());	
	return rand();	
}

void UC_Transmit_Message(char*  message)
{
//	unsigned char exor_data;
	unsigned char length;	
	
	if(!UC_DOH.UART_DO[UC_UART1].Flags.Transmit_Active)
	{			
		//if((message.Lenght < CPK_COMMUNICATION_MIN_PACKAGE_LENGTH) || (message.Lenght > CPK_COMMUNICATION_MAX_PACKAGE_LENGTH))
			//return;
		
		//message.Random_Number = BSL_Get_Random_Number();
			
		//exor_data = message.Random_Number ^ UC_EXOR_FIXED_KEY;
		//length = message.Lenght;
		
	//	message.LRC = UC_Calculate_LRC(&message.Buffer[1],(length - 1));
					
		UC_DOH.UART_DO[UC_UART1].Transmit_Data_Cnt = 0;
		UC_DOH.UART_DO[UC_UART1].Transmit_Data_Total_Cnt = length;	
		
		//memcpy(UC_DOH.UART_DO[UC_UART1].Transmit_Data_Buffer,message.Buffer,length);
		
		UC_DOH.UART_DO[UC_UART1].Flags.Transmit_Active = 1;				
		UC_DOH.UART_DO[UC_UART1].Flags.Tx_Completed = 1;				
	}			
}

/**
 * @brief UART message Buffer send function non polling mode 
 * @param None.
 * @retval None.
 * @note timing detail
 *		function minimum time ==> 1.2us
 *		function maximum time ==> 8.4us (for the 16 byte data)
 */
void UC_Check_Communication(void)
{	
	static uint32_t txTimeout = 0;
	
	if(UC_DOH.UART_DO[UC_UART1].Flags.Transmit_Active)
	{	
		if(UC_DOH.UART_DO[UC_UART1].Flags.Tx_Completed)
		{
			UC_DOH.UART_DO[UC_UART1].Flags.Tx_Completed = 0;
			txTimeout = 0;
			HAL_UART_Transmit_IT(	&HAL_UART1_OBJECT,											/** Send message */
							&UC_DOH.UART_DO[UC_UART1].Transmit_Data_Buffer[UC_DOH.UART_DO[UC_UART1].Transmit_Data_Cnt],
							UC_DOH.UART_DO[UC_UART1].Transmit_Data_Total_Cnt);				
		}
		//zamanlama kontrolü sonrasi 50ms olarak ayarla
		if(txTimeout++ > 10000)
		{
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
			UC_DOH.UART_DO[UC_UART1].Flags.Transmit_Active = 0;								
		}
	}	
	UC_Control_USART_Error();
}

unsigned char UC_Check_Receive(char*  message)
{
	unsigned char loop_cnt;
	unsigned char exor_data;
	unsigned char rxLength;	
	
	if(UC_DOH.UART_DO[UC_UART1].Flags.New_Data_Package_Accepted)
	{				
		rxLength = UC_DOH.UART_DO[UC_UART1].Raw_Buffer_Cnt;
		UC_DOH.UART_DO[UC_UART1].Raw_Buffer_Cnt = 0;		
		
		if((rxLength < 4) || (rxLength > 16)) {
			UC_DOH.UART_DO[UC_UART1].Flags.New_Data_Package_Accepted = 0;
			return 0;
		}
						
		exor_data = UC_DOH.UART_DO[UC_UART1].Raw_Buffer[1] ^ UC_EXOR_FIXED_KEY;
		
		for(loop_cnt = 2; loop_cnt < (rxLength) ; loop_cnt++)
			UC_DOH.UART_DO[UC_UART1].Raw_Buffer[loop_cnt] ^= exor_data;
		
		if(UC_DOH.UART_DO[UC_UART1].Raw_Buffer[0] == UC_Calculate_LRC(&UC_DOH.UART_DO[UC_UART1].Raw_Buffer[1],(rxLength-1)))
		{
			//memcpy(message->Buffer,UC_DOH.UART_DO[UC_UART1].Raw_Buffer,rxLength);			
			UC_DOH.UART_DO[UC_UART1].Flags.New_Data_Package_Accepted = 0;
			return 1;
		}
		else {
			UC_DOH.UART_DO[UC_UART1].Flags.New_Data_Package_Accepted = 0;
			return 0;			
		}
	}
	else {
		UC_DOH.UART_DO[UC_UART1].Flags.New_Data_Package_Accepted = 0;
		return 0;
	}
}

void UC_IRQ_Virtual_RX(unsigned char UART_No,unsigned char UART_RXN)
{
	
	if(UC_DOH.UART_DO[UC_UART1].Flags.New_Data_Package_Accepted == 1)
	{
		return;
	}
	
	
	UC_DOH.UART_DO[UART_No].Raw_Buffer[UC_DOH.UART_DO[UART_No].Raw_Buffer_Cnt] = UART_RXN;	 		/** Data write the UART raw buffer */
	UC_DOH.UART_DO[UART_No].Raw_Buffer_Cnt = (UC_DOH.UART_DO[UART_No].Raw_Buffer_Cnt < UART_RAW_BUFFER_SIZE ? UC_DOH.UART_DO[UART_No].Raw_Buffer_Cnt + 1 : UC_DOH.UART_DO[UART_No].Raw_Buffer_Cnt ); /** increase the raw buffer counter */
	UC_DOH.UART_DO[UART_No].Flags.New_Data_Start = 1;								/** set new data flag*/
	UC_DOH.UART_DO[UART_No].Data_End_Off_Time_Counter = 0;							/** reset end of time counter */
}

void UC_IRQ_Virtual_Systick(void)
{
	for (int UART = 0; UART < UC_TOTAL_UART_COUNT; ++UART)
	{
		if(UC_DOH.UART_DO[UART].Flags.New_Data_Start)										/** check is new data coming */
		{
			if(++UC_DOH.UART_DO[UART].Data_End_Off_Time_Counter > UART_END_OFF_TIME)		/** check end of time for message */
			{								
				UC_DOH.UART_DO[UART].Flags.New_Data_Package_Accepted = 1;					/** set new data accepted flag  */
				UC_DOH.UART_DO[UART].Flags.New_Data_Start = 0;							/** clear new message coming flag */
				UC_DOH.UART_DO[UART].Data_End_Off_Time_Counter = 0;						/** clear data end of time counter */
			}
		}
	}
}

/*** (C) COPYRIGHT KRNK AR-GE Müh. ve Dan. Hizm. LTD. STI. *****END OF FILE****/
