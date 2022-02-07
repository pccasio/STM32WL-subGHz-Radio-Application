/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio_driver.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STATE_STANDBY,
  STATE_TX
} state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RF_FREQUENCY                                868000000 /* Hz */
#define TX_OUTPUT_POWER                             14        /* dBm */
#define FSK_FDEV                                    25000     /* Hz */
#define FSK_DATARATE                                50000     /* bps */
#define FSK_BANDWIDTH                               50000     /* Hz */
#define FSK_PREAMBLE_LENGTH                         5         /* Same for Tx and Rx */
#define FSK_SYNCWORD_LENGTH                         3
#define PAYLOAD_LEN                                 0x4        

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Radio events function pointer */

PacketParams_t packetParams; //why here???

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void loadBuffer(uint8_t* buffer, uint8_t* data, int data_length);
void radioInit();
void sendData(uint8_t* payload, uint8_t payloadLen);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);

/*

void eventRxDone(pingPongFSM_t *const fsm);
void eventRxTimeout(pingPongFSM_t *const fsm);
void eventRxError(pingPongFSM_t *const fsm);
void enterMasterRx(pingPongFSM_t *const fsm);
void enterSlaveRx(pingPongFSM_t *const fsm);

void enterSlaveTx(pingPongFSM_t *const fsm);
void transitionRxDone(pingPongFSM_t *const fsm);
*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char uartBuffer[100];
	uint8_t dataBuffer[32];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	radioInit();
	uint8_t payload[] = "PING";
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	RadioStatus_t status;
	uint8_t irqStatus;
	
  while (1)
  {
    /* USER CODE END WHILE */
			
    /* USER CODE BEGIN 3 */
    
	/*irqStatus = SUBGRF_GetIrqStatus();
	sprintf(uartBuffer, "IrqStatus=%d \r\n", irqStatus);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
	*/
		sendData(payload, sizeof(payload)-1);
		status = SUBGRF_GetStatus();

		sprintf(uartBuffer, "Operating Mode=%d \r\n", SUBGRF_GetOperatingMode());
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
		
		HAL_Delay(2000);
  }
  /* USER CODE END 3 */ 
	
	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
void loadBuffer(uint8_t* buffer, uint8_t* data, int data_length){
	int i;
	for(i=0; i<data_length; i++){
		buffer[i] = data[i];
	}
}
*/

void sendData(uint8_t* payload, uint8_t payloadLen){
	SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE);
	
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
	packetParams.Params.Gfsk.PayloadLength = 0x4;
  SUBGRF_SetPacketParams(&packetParams);
	/*uint8_t buffer[32]; 
	//loadBuffer(buffer, payload, sizeof(payload)-1); //eol character
	loadBuffer(buffer, payload, payloadLen);
	SUBGRF_WriteBuffer(0, buffer, payloadLen);
	*/
	SUBGRF_SetPayload( payload, 4);
	SUBGRF_SetTx(0);
}

void radioInit(){
	
	SUBGRF_Init(RadioOnDioIrq);
	
	//SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
  //SUBGRF_SetRegulatorMode();
	
	SUBGRF_SetBufferBaseAddress(0x00, 0x00); //NOT THIS
	
	SUBGRF_SetRfFrequency(RF_FREQUENCY); //NOT THESE
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
  //SUBGRF_SetStopRxTimerOnPreambleDetect(false);
	
	ModulationParams_t modParams;
	modParams.PacketType = PACKET_TYPE_GFSK;
	modParams.Params.Gfsk.Bandwidth = SUBGRF_GetFskBandwidthRegValue(FSK_BANDWIDTH);
	modParams.Params.Gfsk.BitRate = FSK_DATARATE;
	modParams.Params.Gfsk.Fdev = FSK_FDEV;
	modParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
	SUBGRF_SetModulationParams(&modParams);
	
	
	packetParams.PacketType = PACKET_TYPE_GFSK;
	packetParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
	packetParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
	packetParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;
	packetParams.Params.Gfsk.HeaderType = RADIO_PACKET_VARIABLE_LENGTH;
	packetParams.Params.Gfsk.PayloadLength = PAYLOAD_LEN;
	packetParams.Params.Gfsk.PreambleLength = (FSK_PREAMBLE_LENGTH << 3); //bytes to bits!!
	packetParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
	packetParams.Params.Gfsk.SyncWordLength = (FSK_SYNCWORD_LENGTH << 3);
	SUBGRF_SetPacketParams(&packetParams);
	
	//SUBGRF_SetRfFrequency(RF_FREQUENCY);
	SUBGRF_SetSyncWord((uint8_t[]){0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00});
	//SUBGRF_SetWhiteningSeed(0x01FF);
	//SUBGRF_SetTxParams(RFO_LP, 14, RADIO_RAMP_40_US);
	
	//SUBGRF_SetPaConfig();
	//SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
}

// Callback function - executed once for each (enabled) interrupt issued from sub-GHz radio
void RadioOnDioIrq(RadioIrqMasks_t radioIrq){
  switch (radioIrq){
    case IRQ_TX_DONE:
      //HAL_UART_Transmit(&huart2, (uint8_t *)"TX done int\r\n", 13, HAL_MAX_DELAY);
      break;
    //case IRQ_RX_DONE:
      //break;
    case IRQ_RX_TX_TIMEOUT:

      break;
    case IRQ_CRC_ERROR:
      break;
    default:
      break;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
