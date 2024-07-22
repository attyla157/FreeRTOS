/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for UART_RX_task */
osThreadId_t UART_RX_taskHandle;
const osThreadAttr_t UART_RX_task_attributes = {
  .name = "UART_RX_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_TX_task */
osThreadId_t UART_TX_taskHandle;
const osThreadAttr_t UART_TX_task_attributes = {
  .name = "UART_TX_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MessageEcuator */
osThreadId_t MessageEcuatorHandle;
const osThreadAttr_t MessageEcuator_attributes = {
  .name = "MessageEcuator",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MeasureLoop */
osThreadId_t MeasureLoopHandle;
const osThreadAttr_t MeasureLoop_attributes = {
  .name = "MeasureLoop",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LedControlTask */
osThreadId_t LedControlTaskHandle;
const osThreadAttr_t LedControlTask_attributes = {
  .name = "LedControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MsgQueue */
osMessageQueueId_t MsgQueueHandle;
const osMessageQueueAttr_t MsgQueue_attributes = {
  .name = "MsgQueue"
};
/* Definitions for TxQueue */
osMessageQueueId_t TxQueueHandle;
const osMessageQueueAttr_t TxQueue_attributes = {
  .name = "TxQueue"
};
/* Definitions for RxCompleteFlag */
osEventFlagsId_t RxCompleteFlagHandle;
const osEventFlagsAttr_t RxCompleteFlag_attributes = {
  .name = "RxCompleteFlag"
};
/* Definitions for LedFlag */
osEventFlagsId_t LedFlagHandle;
const osEventFlagsAttr_t LedFlag_attributes = {
  .name = "LedFlag"
};
/* Definitions for MeasureFlag */
osEventFlagsId_t MeasureFlagHandle;
const osEventFlagsAttr_t MeasureFlag_attributes = {
  .name = "MeasureFlag"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void Start_UART_RX_task(void *argument);
void Start_UART_TX_task(void *argument);
void StartMessageEcuator(void *argument);
void StartMeasureLoop(void *argument);
void StartLedControlTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__io_putchar(int ch)
{
	ITM_SendChar(ch);
}
struct message_struct{

	char character[20];
};

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MsgQueue */
  MsgQueueHandle = osMessageQueueNew (20, sizeof(struct message_struct), &MsgQueue_attributes);

  /* creation of TxQueue */
  TxQueueHandle = osMessageQueueNew (20, sizeof(struct message_struct), &TxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_RX_task */
  UART_RX_taskHandle = osThreadNew(Start_UART_RX_task, NULL, &UART_RX_task_attributes);

  /* creation of UART_TX_task */
  UART_TX_taskHandle = osThreadNew(Start_UART_TX_task, NULL, &UART_TX_task_attributes);

  /* creation of MessageEcuator */
  MessageEcuatorHandle = osThreadNew(StartMessageEcuator, NULL, &MessageEcuator_attributes);

  /* creation of MeasureLoop */
  MeasureLoopHandle = osThreadNew(StartMeasureLoop, NULL, &MeasureLoop_attributes);

  /* creation of LedControlTask */
  LedControlTaskHandle = osThreadNew(StartLedControlTask, NULL, &LedControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of RxCompleteFlag */
  RxCompleteFlagHandle = osEventFlagsNew(&RxCompleteFlag_attributes);

  /* creation of LedFlag */
  LedFlagHandle = osEventFlagsNew(&LedFlag_attributes);

  /* creation of MeasureFlag */
  MeasureFlagHandle = osEventFlagsNew(&MeasureFlag_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *handle)
{
	osEventFlagsSet(RxCompleteFlagHandle, 0x1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_UART_RX_task */
/**
  * @brief  Function implementing the UART_RX_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_UART_RX_task */
void Start_UART_RX_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	uint8_t buffor[20]={0};
	uint8_t temp = 0;
	uint8_t index = -1 ;
	HAL_StatusTypeDef RxStatus;
	struct message_struct wiadomosc= {{0}};
  for(;;)
  {
	  RxStatus =  HAL_UART_Receive_IT(&huart2, &temp, 1);
	  osEventFlagsWait(RxCompleteFlagHandle, 0x1,osFlagsWaitAll , osWaitForever );
	  //HAL_UARTEx_ReceiveToIdle_IT(&huart2, buffor, 20);
	  index ++;
	  wiadomosc.character[index]= temp;
	  if ((temp  == 0x0a) || (index == 19))
		  {

		  index = -1;
		  osEventFlagsSet(RxCompleteFlagHandle, 0x2);
		  RxStatus =osMessageQueuePut(MsgQueueHandle, &wiadomosc, 1, osWaitForever);
		 memset(wiadomosc.character,0,sizeof(wiadomosc.character));

		  }
	  osEventFlagsClear(RxCompleteFlagHandle, 0x1);

	  //osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_UART_TX_task */
/**
* @brief Function implementing the UART_TX_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART_TX_task */
void Start_UART_TX_task(void *argument)
{
  /* USER CODE BEGIN Start_UART_TX_task */
  /* Infinite loop */
	uint8_t buffor[20] = {0};
	struct message_struct wiadomosc= {{0}};
		  /* Infinite loop */
		  for(;;)
		  {
			  osEventFlagsWait(RxCompleteFlagHandle, 0x4, osFlagsWaitAll, osWaitForever);
			  osMessageQueueGet(TxQueueHandle, &wiadomosc, 1, 1000);
			  HAL_UART_Transmit_IT(&huart2, wiadomosc.character, 20);
			  osEventFlagsClear(RxCompleteFlagHandle, 0x4);

		  }
  /* USER CODE END Start_UART_TX_task */
}

/* USER CODE BEGIN Header_StartMessageEcuator */
/**
* @brief Function implementing the MessageEcuator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMessageEcuator */
void StartMessageEcuator(void *argument)
{
  /* USER CODE BEGIN StartMessageEcuator */
  /* Infinite loop */
	uint8_t buffor[20] = {0};
			struct message_struct wiadomosc = {{0}};
			uint8_t stage = -1;
		  /* Infinite loop */
		  for(;;)
		  {
			 // printf("%d",osMessageQueueGetCount(MsgQueueHandle));
			  osEventFlagsWait(RxCompleteFlagHandle, 0x2, osFlagsWaitAll, osWaitForever);
			  osMessageQueueGet(MsgQueueHandle,&wiadomosc, 1, 1000);


			  if(!strncmp(wiadomosc.character,"MEASURE\n\0",7))
			  {
				  stage = 0;
			  }

			  else if(!strcmp(wiadomosc.character,"PWR\n\0"))
			  {
				  stage = 1;
			  }
			  else if(!strcmp(wiadomosc.character,"LED STOP\n\0"))
			  {
				  osEventFlagsClear(LedFlagHandle, 0xffffffff);
				  osEventFlagsSet(LedFlagHandle, 0x1);
				  stage = 2;
			  }
			  else if(!strcmp(wiadomosc.character,"LED START\n\0"))
			  {
				  stage = 3;
				  osEventFlagsClear(LedFlagHandle, 0xffffffff);
				  osEventFlagsSet(LedFlagHandle, 0x3);
			  }
			  else
			  {
				  stage = -1;
			  }
			  osMessageQueuePut(TxQueueHandle, &wiadomosc, 1, 1000);
			  osEventFlagsClear(RxCompleteFlagHandle, 0x2);
			  osEventFlagsSet(RxCompleteFlagHandle, 0x4);

		    osDelay(100);
		  }
  /* USER CODE END StartMessageEcuator */
}

/* USER CODE BEGIN Header_StartMeasureLoop */
/**
* @brief Function implementing the MeasureLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMeasureLoop */
void StartMeasureLoop(void *argument)
{
  /* USER CODE BEGIN StartMeasureLoop */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMeasureLoop */
}

/* USER CODE BEGIN Header_StartLedControlTask */
/**
* @brief Function implementing the LedControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedControlTask */
void StartLedControlTask(void *argument)
{
  /* USER CODE BEGIN StartLedControlTask */
  /* Infinite loop */
	int flags = 0;
  for(;;)
  {
	  osEventFlagsWait(LedFlagHandle, 0x1, osFlagsWaitAny, osWaitForever);
	 flags =  osEventFlagsGet(LedFlagHandle);
	 if(flags == 0)
	 {
		 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
		 osEventFlagsClear(LedFlagHandle, 0x1);
	 }
	 else if(flags == 2)
	 {
		 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
		 osEventFlagsClear(LedFlagHandle, 0x2);
	 }

	 flags =  osEventFlagsGet(LedFlagHandle);
    osDelay(100);
  }
  /* USER CODE END StartLedControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
