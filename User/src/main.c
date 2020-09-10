/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "nrf24l01.h"
#include "user_config.h"
#include "main.h"

//#define USE_STDIO

/** @addtogroup Template
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f429_439xx.s) before to branch to application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */  

	/* Initialize uart for baud rate: 115200 */
	UART_Init();
    
    debugs("\n\n\n\r\n");

	/* Initialize system delay */
    debugs("System delay initial...\t\t");
	Delay_Init(1000);
    debugs(GRN_BOLD"[ok!]\r\n"ARESET);

	/* Initial GPIO_LED */
    debugs("GPIO LED initial...\t\t");
	LED_Init(LED_BOTH);

	/* Turn off LED first */
	STM_LED_OFF(LED_BOTH);
    debugs(GRN_BOLD"[ok!]\r\n"ARESET);

	debugs("Start initial NRF24L01...\t");

	/** Initialize NRF24L01+ on channel 15 and 32bytes of payload.
	 *  By default 2Mbps data rate and 0dBm output power.
	 *  NRF24L01 goes to RX mode by default
	 */
	NRF24L01_Init(76, 20);

	/* Set RF settings, Data rate to 2Mbps, Output power to -18dBm */
	NRF24L01_SetRF(NRF24L01_DataRate_1M, NRF24L01_OutputPower_M18dBm);

	/* Set Rx addresses */
	NRF24L01_SetRxAddress(0, (uint8_t*)"0pipe");
	NRF24L01_SetRxAddress(1, (uint8_t*)"1pipe");
	NRF24L01_SetRxAddress(2, (uint8_t*)"2");
	NRF24L01_SetRxAddress(3, (uint8_t*)"3");
	NRF24L01_SetRxAddress(4, (uint8_t*)"4");
	NRF24L01_SetRxAddress(5, (uint8_t*)"5");

	/* Set TX address, 5 bytes */
	NRF24L01_SetTxAddress((uint8_t*)"gaway");

    debugs(GRN_BOLD"[ok!]\r\n"ARESET);

	uint8_t dataIn[32];
	NRF24L01_Transmit_Status_t transmissionStatus;

	debugs("NRF24L01 Starting Listening...\r\n");
  print_details();


	/* Infinite loop */
	while (1)
	{
		/* If data is ready on NRF24L01+ */
		if (NRF24L01_DataReady()) {
			/* Get data from NRF24L01+ */
			NRF24L01_GetData(dataIn);

			debugs("Got payload: %s\r\n", dataIn);
     
			/* Start send */
			STM_LED_ON(LED_GREEN);

			/* Send it back, automatically goes to TX mode */
			NRF24L01_Transmit(dataIn);

			/* Wait for data to be sent */
			do {
				/* Wait till sending */
				transmissionStatus = NRF24L01_GetTransmissionStatus();
			} while (transmissionStatus == NRF24L01_Transmit_Status_Sending);

			/* Send done */
			STM_LED_OFF(LED_GREEN);

			/* Go back to RX mode */
		    NRF24L01_PowerUpRx();
		}
	}

    /* Would not running here */
    debugs("[Failed] System starting failed!!\r\n");
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
      debugs(RED_BOLD"[Assert FAILED]"ARESET);
      debugs(" File: %s, Line at: %d\r\n", (char)file, (int)line);
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
