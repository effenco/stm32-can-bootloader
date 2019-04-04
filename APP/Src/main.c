/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IO_MGMT.h"

#include "globalvars.h"
#include "globaldefines.h"

#include "string.h"
#include "CANTX_FIFO.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t FORCE_CAL_BIT=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void config_can_filter(void);
void CAN_TX_Cplt(CAN_HandleTypeDef* phcan);

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
  MX_CAN_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

	/* Configure CAN RX FILTER */
	config_can_filter();

  	/* Initialize CAN TX FIFO */
	CANTX_FIFO_init(&CanTxList,&hcan);

	/* Start Timers */
	HAL_TIM_Base_Start_IT(&htim17); // 1mS

	/* Start CAN recieve Interrupt */
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
  }

// Keep hardware/software compatibility between versions
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 32000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_R_GPIO_Port, GPIO_LED1_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_LED1_G_Pin|GPIO_LED1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_LED1_R_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_LED1_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_G_Pin GPIO_LED1_B_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_G_Pin|GPIO_LED1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* Interrupt callback functions-----------------------------------------------*/

/* HAL_TIM_PeriodElapsedCallback ---------------------------------------------*/
/* Interrupt callback to manage timer interrupts							  */
/* htim19 1ms																  */
/*----------------------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* htim17 1ms */
	if (&htim17 == htim) {

		static uint16_t cpt2=0;

		//SDADC_SAMPLE_ALL();	// Sample all channels 1mS oversampling

		G_mSCounter++;

		if(G_mSCounter>=G_MAX_MS_COUNTER){
			G_mSCounter=0;
		}

		cpt2++;

		/* 1 S, reinitiate cpt2 */
		if(cpt2>=1000){
			cpt2=0;
		}

		/* 10 mS */
		if(cpt2%10==0){
			/* Main loop -----------------------------------------------------*/

			/* Input Read ------------------------------------------------*/

			/*------------------------------------------------ Input Read */

			/* Control loop ----------------------------------------------*/

			/*---------------------------------------------- Control loop */

			/* Output Write ----------------------------------------------*/

			// Mid layer output management

			// LED CONTROL
				if (G_mSCounter % 250 == 0) {
					// every 500mS update the LED Status
					static uint8_t led_state = 1;

					switch (led_state) {
					case 1:
						HAL_GPIO_WritePin(GPIO_LED1_R_GPIO_Port,
								GPIO_LED1_R_Pin, 1);
						HAL_GPIO_WritePin(GPIO_LED1_G_GPIO_Port,
								GPIO_LED1_G_Pin, 0);
						HAL_GPIO_WritePin(GPIO_LED1_B_GPIO_Port,
								GPIO_LED1_B_Pin, 0);
						led_state++;
						break;
					case 2:
						HAL_GPIO_WritePin(GPIO_LED1_R_GPIO_Port,
								GPIO_LED1_R_Pin, 0);
						HAL_GPIO_WritePin(GPIO_LED1_G_GPIO_Port,
								GPIO_LED1_G_Pin, 1);
						HAL_GPIO_WritePin(GPIO_LED1_B_GPIO_Port,
								GPIO_LED1_B_Pin, 0);
						led_state++;
						break;
					case 3:
						HAL_GPIO_WritePin(GPIO_LED1_R_GPIO_Port,
								GPIO_LED1_R_Pin, 0);
						HAL_GPIO_WritePin(GPIO_LED1_G_GPIO_Port,
								GPIO_LED1_G_Pin, 0);
						HAL_GPIO_WritePin(GPIO_LED1_B_GPIO_Port,
								GPIO_LED1_B_Pin, 1);
						led_state = 1;
						break;
					default:
						break;
					}
				}

			/* Send 1000mS CAN messages */
			if (cpt2 == 0) {
				/* Add CAN messages to FIFO */
				CAN_SEND_1000ms();
			}

			/* Send 100mS CAN messages */
			if (cpt2 % 100 == 0) {
				/* Add CAN messages to FIFO */
				CAN_SEND_100ms();
			}

			/* Send 20mS CAN messages */
			if (cpt2 % 20 == 0) {
				/* Add CAN messages to FIFO */
				CAN_SEND_20ms();
			}

			/* Initiate CAN transmit */
			if(CanTxList.current_qty_in_queue!=0){
				CANTX_FIFO_INITIATE_TRANSMIT(&CanTxList, &hcan);
			}
			/*---------------------------------------------- Output Write */

			/*----------------------------------------------------- Main loop */
		}

	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *phcan){
	uint32_t transmitmailbox;
	int retry=0;

	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX0);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST1) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR1)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX1);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST2) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR2)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX2);
		retry=1;
	}

	HAL_CAN_ResetError(phcan);

	if(retry==1){
		HAL_CAN_DeactivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_ActivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_AddTxMessage(phcan,&(CanTxList.SendMsgBuff.header),CanTxList.SendMsgBuff.Data,&transmitmailbox);
	}
}

/* HAL_CAN_TxCpltCallback ----------------------------------------------------*/
/* Interrupt callback to manage Can Tx Ready                                  */
/*----------------------------------------------------------------------------*/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}

void CAN_TX_Cplt(CAN_HandleTypeDef* phcan){
	uint32_t transmitmailbox;
		if (CanTxList.current_qty_in_queue) {
			CanTxList.SendMsgBuff = CANTX_FIFO_pull(&CanTxList);

			if(CanTxList.SendMsgBuff.header.ExtId!=0){

				HAL_CAN_AddTxMessage(phcan,&(CanTxList.SendMsgBuff.header),CanTxList.SendMsgBuff.Data,&transmitmailbox);
			}
		} else {
			CanTxList.TxInProgress = 0;
		}
}
/*---------------------------------------------------- HAL_CAN_TxCpltCallback */

/* HAL_CAN_RxCpltCallback ----------------------------------------------------*/
/* Interrupt callback to manage Can Rx message ready to read                  */
/*----------------------------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* phcan){

	CAN_RxHeaderTypeDef rxheader;
	uint8_t data[8];

	HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO0, &rxheader, data);

	/* Low level Can management ----------------------------------------------*/

}
/*---------------------------------------------------- HAL_CAN_RxCpltCallback */

void config_can_filter(void){
  CAN_FilterTypeDef sFilterConfig;

  uint32_t filterMask= 0xFFFF88FF;
  uint32_t filterID=   0x18FF00E7; // Only accept ECU CAN message ID

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = filterID >> 13;
  sFilterConfig.FilterIdLow = (0x00FF & (filterID << 3)) | (1 << 2);;
  sFilterConfig.FilterMaskIdHigh = filterMask >> 13;
  sFilterConfig.FilterMaskIdLow = (0x00FF & (filterMask << 3)) | (1 << 2);
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 0;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
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
  while(1)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
