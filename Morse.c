/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <FreeRTOS.h>

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
/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task03 */
osThreadId_t Task03Handle;
const osThreadAttr_t Task03_attributes = {
  .name = "Task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task04 */
osThreadId_t Task04Handle;
const osThreadAttr_t Task04_attributes = {
  .name = "Task04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Queue1 */
osMessageQueueId_t Queue1Handle;
const osMessageQueueAttr_t Queue1_attributes = {
  .name = "Queue1"
};
/* Definitions for queueISRHandle */
osMessageQueueId_t queueISRHandleHandle;
const osMessageQueueAttr_t queueISRHandle_attributes = {
  .name = "queueISRHandle"
};
/* Definitions for Queue2_1 */
osMessageQueueId_t Queue2_1Handle;
const osMessageQueueAttr_t Queue2_1_attributes = {
  .name = "Queue2_1"
};
/* Definitions for Queue2_2 */
osMessageQueueId_t Queue2_2Handle;
const osMessageQueueAttr_t Queue2_2_attributes = {
  .name = "Queue2_2"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

int _write(int file, char *ptr, int len)
{
	int i = 0;
	for (i=0; i < len; i++)
	{
		ITM_SendChar((*ptr++));
	}
	return len;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t currentTick;
	BaseType_t xHigherPriorityTaskWoken;


	currentTick = xTaskGetTickCountFromISR();
	xHigherPriorityTaskWoken = pdFALSE;

	//loop until buffer is empty
	xQueueSendToBackFromISR(queueISRHandleHandle, &currentTick, &xHigherPriorityTaskWoken);

	 portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
  /* creation of Queue1 */
  Queue1Handle = osMessageQueueNew (16, sizeof(uint32_t), &Queue1_attributes);

  /* creation of queueISRHandle */
  queueISRHandleHandle = osMessageQueueNew (16, sizeof(uint32_t), &queueISRHandle_attributes);

  /* creation of Queue2_1 */
  Queue2_1Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue2_1_attributes);

  /* creation of Queue2_2 */
  Queue2_2Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue2_2_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(StartTask01, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

  /* creation of Task03 */
  Task03Handle = osThreadNew(StartTask03, NULL, &Task03_attributes);

  /* creation of Task04 */
  Task04Handle = osThreadNew(StartTask04, NULL, &Task04_attributes);
  vTaskSuspend(Task04Handle);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  uint32_t DataToReceive = 0;
  uint32_t DataToSend = 0;
  BaseType_t xHigherPriorityTaskWoken;
  uint32_t lastTick = 0;
  uint32_t onePress = 0;
  uint32_t press = 0;
  uint32_t release = 0;
  uint32_t timeOfPress = 0;
  uint32_t lastPress = 0;
  uint32_t timeTillDeath = 10000;
  for(;;)
  {
	  if (uxQueueMessagesWaiting(queueISRHandleHandle) != 0)
	  {
		  xHigherPriorityTaskWoken = pdFALSE;
		  xQueueReceiveFromISR(queueISRHandleHandle, &DataToReceive, &xHigherPriorityTaskWoken);
		  //printf("current Tick = %" PRIu32 "\n", DataToReceive);
		  if (onePress == 0)
			  press = DataToReceive;
		  else if(onePress == 1)
			  release = DataToReceive;
		  onePress++;
		  lastPress = xTaskGetTickCountFromISR();
		  timeTillDeath = lastPress + 10000;

	  }
	  if(onePress == 2)
	  {
		  timeOfPress = release - press;
		  onePress = 0;

		  if(DataToReceive - lastTick >= 5000)
		  {
			  //printf("Space between words\n");
			  DataToSend = 5000;
			  xHigherPriorityTaskWoken = pdFALSE;
			  xQueueSendToBackFromISR(Queue1Handle, &DataToSend, &xHigherPriorityTaskWoken);
		  }
		  else if(DataToReceive - lastTick >= 2500)
		  {
			  //printf("space between letters\n");
			  DataToSend = 1500;
			  xHigherPriorityTaskWoken = pdFALSE;
			  xQueueSendToBackFromISR(Queue1Handle, &DataToSend, &xHigherPriorityTaskWoken);
		  }
		  if(timeOfPress < 300)
		  {
			  //printf("That was a dot\n");
			  DataToSend = 300;
			  xHigherPriorityTaskWoken = pdFALSE;
			  xQueueSendToBackFromISR(Queue1Handle, &DataToSend, &xHigherPriorityTaskWoken);
		  }
		  else
		  {
			  //printf("that was a dash\n");
			  DataToSend = 700;
			  xHigherPriorityTaskWoken = pdFALSE;
			  xQueueSendToBackFromISR(Queue1Handle, &DataToSend, &xHigherPriorityTaskWoken);
		  }

		  lastTick = DataToReceive;
	  }
	  if(xTaskGetTickCountFromISR() > timeTillDeath)
	  {
		  printf("Suspending Tasks\n");
		  vTaskSuspend(Task03Handle);
		  vTaskSuspend(Task02Handle);
		  xTaskResumeFromISR(Task04Handle);
		  vTaskSuspend(Task01Handle);

	  }
	  vTaskDelay(1);

  }
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  uint32_t DataToReceive = 0;
  //TickType_t delay = 0;
  BaseType_t xHigherPriorityTaskWoken;
  for(;;)
  {
	  if (uxQueueMessagesWaiting(Queue1Handle) != 0)
	  {
		  xHigherPriorityTaskWoken = pdFALSE;
		  xQueueReceiveFromISR(Queue1Handle, &DataToReceive, &xHigherPriorityTaskWoken);
		  if(DataToReceive == 300)
		  {
			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			  vTaskDelay(300);
			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		  }
		  else if(DataToReceive == 700)
		  {
			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			  vTaskDelay(700);
			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		  }
		  //printf("Task 2 sending %" PRIu32 "\n", DataToReceive);
		  xQueueSendToBackFromISR(Queue2_1Handle, &DataToReceive, &xHigherPriorityTaskWoken);
		  xQueueSendToBackFromISR(Queue2_2Handle, &DataToReceive, &xHigherPriorityTaskWoken);
	  }

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  uint32_t DataToReceive = 0;
  BaseType_t xHigherPriorityTaskWoken;
  int i = 0;
  uint32_t input[4];
  bool endOfLetter = false;
  for(;;)
  {
	  if (uxQueueMessagesWaiting(Queue2_1Handle) != 0)
	  	  {
		  	  //printf("found message task\n");
	  		  xHigherPriorityTaskWoken = pdFALSE;
	  		  xQueueReceiveFromISR(Queue2_1Handle, &DataToReceive, &xHigherPriorityTaskWoken);
	  		  //printf("data %" PRIu32 "\n", DataToReceive);
	  		  if((DataToReceive == 300) || (DataToReceive == 700))
	  		  {
	  			  input[i]=DataToReceive;

	  			  //printf("sent data to array %d\n", i);
	  			  //printf("array:  %" PRIu32 "\n", input[i]);
	  			  i++;
	  		  }
	  		  else if(DataToReceive == 1500)
	  		  {
	  			  endOfLetter = true;
	  		  }
	  		  else if(DataToReceive == 5000)
	  		  {
	  			  endOfLetter = true;
	  			  printf(" ");
	  		  }
	  		  if((i == 3) || (endOfLetter))
	  		  {
				  if(input[0] == 300)//300
				  {
					 if(input[1] == 300)//300300
					 {
						 if(input[2] == 300)//300300300
						 {
							if(input[3]==NULL)//300300300null
								printf("S ");
							else if(input[3]==300)//300300300300
								printf("H");
							else if(input[3]==700)//300300300700
								printf("V");
						 }
						 else if(input[2] == 700)//300300700
						 {
							if(input[3]==NULL)//300300700null
								printf("U");
							else if(input[3]==300)//300300700300
								printf("F");
						 }
						 else if(input[2] == NULL)//300300null
							printf("I");
					 }
					 else if(input[1] == 700)//300700
					 {
						if(input[2] == NULL)//300700Null
							printf("A");
						else if(input[2] == 700)//300700700
						{
							if(input[3]==NULL)//300700700null
								printf("W");
							else if(input[3]==300)//300700700300
								printf("P");
							else if(input[3]==700)//300700700700
								printf("J");
						}
						else if(input[2]==300)//300700300
						{
							if(input[3]==NULL)//300700300null
								printf("R");
							if(input[3]==300)//300700300300
								printf("L");
						}
					 }
					 else if(input[1] == NULL)//300null
						printf("E");
				  }
				  else if(input[0] == 700) //700
				  {
					  if(input[1] == 300)//700300
					  {
						  if(input[2]==NULL)//700300null
							  printf("N");
						  else if(input[2] == 700)//700300700
						  {
							  if(input[3] == 300)//700300700300
								 printf("C");
							  else if(input[3]==700)//700300700700
								  printf("Y");
							  else if(input[3] == NULL)//700300700NULL
								  printf("K");
						  }
						  else if(input[2]==300)//700300300
						  {
							  if(input[3]==NULL)//700300300null
								  printf("D");
							  if(input[3]==300)//700300300300
								  printf("B");
							  else if(input[3]==700)//700300300700
								 printf("X");
						  }
					  }
					else if(input[1] == 700)//700700
					{
						if(input[2]==NULL)//700700null
							printf("M");
						else if(input[2] == 700)//700700700
						{
							if(input[3] == NULL)//700700700null
								printf("O");
						}
						else if (input[2]==300)//700700300
						{
							if(input[3]==NULL)//700700300null
								printf("G");
							else if(input[3]==300)//700700300300
								printf("Z");
							else if(input[3]==700)//700700300700
								printf("Q");
						}
					}
					else if(input[1] == NULL) //700null
						printf("T");
				  }
				  else if(input[0] == NULL)
					  printf("");
		i = 0;
		endOfLetter = false;
		for (int x = 0; x < 4; x++)
			input[x] = NULL;
	  	}
	}
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  uint32_t DataToReceive = 0;
  //TickType_t delay = 0;
  BaseType_t xHigherPriorityTaskWoken;
  printf("Starting Task4");
  for(;;)
  {
	  if (uxQueueMessagesWaiting(Queue2_2Handle) != 0)
	  	  {
	  		  xHigherPriorityTaskWoken = pdFALSE;
	  		  xQueueReceiveFromISR(Queue2_2Handle, &DataToReceive, &xHigherPriorityTaskWoken);
	  		  if(DataToReceive == 300)
	  		  {
	  			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  			  vTaskDelay(300);
	  			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  			  vTaskDelay(300);
	  		  }
	  		  else if(DataToReceive == 700)
	  		  {
	  			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  			  vTaskDelay(700);
	  			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  			  vTaskDelay(700);
	  		  }
	  		  else if(DataToReceive == 1500)
	  		  {
	  			  vTaskDelay(1500);
	  		  }
	  		  else if(DataToReceive == 5000)
	  		  {
	  			  vTaskDelay(5000);
	  		  }
	  	  }
  }
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
