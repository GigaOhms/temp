/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "led.h"
#include "giga_func.h"
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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

osThreadId WriteGPIO_PWMHandle;
osThreadId ReadAnalogSensoHandle;
osThreadId ReadGyroSensorHandle;

/* USER CODE BEGIN PV */

uint8_t start_run = 0;
volatile uint16_t analog_val[10];
uint8_t but_mode = 0;
uint8_t but_select = 0;
uint8_t cnt_mode = 0;
uint8_t cnt_select = 0;

		uint8_t data_gyro[6];
		volatile int16_t x_gyro;
		volatile int16_t y_gyro;
		volatile int16_t z_gyro;

/*
void bzEnable(void);
void mForward(void);
void mBackward(void);
void mStop(void);
void mLeft(void);
void mRight(void);
void loop_7seg(void);
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void Write_IO_PWM(void const * argument);
void Read_Analog_Ssr(void const * argument);
void Read_Gyro_Ssr(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_2);	
	
	/*
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (0x68<<1) + 0, 1, 100);
	if (ret == HAL_OK)
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
	
	uint8_t temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, (0x68<<1) + 0, 27, 1, &temp_data, 1, 100);
	if (ret == HAL_OK) {
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
	
	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, (0x68<<1) + 0, 28, 1, &temp_data, 1, 100);
	if (ret == HAL_OK) {
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
	
	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, (0x68<<1) + 0, 107, 1, &temp_data, 1, 100);
	if (ret == HAL_OK) {
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
	*/
	
	
	HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_SET);
	Show_7Seg(0);
	stop_motor();
	
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of WriteGPIO_PWM */
  osThreadDef(WriteGPIO_PWM, Write_IO_PWM, osPriorityLow, 0, 128);
  WriteGPIO_PWMHandle = osThreadCreate(osThread(WriteGPIO_PWM), NULL);

  /* definition and creation of ReadAnalogSenso */
  osThreadDef(ReadAnalogSenso, Read_Analog_Ssr, osPriorityBelowNormal, 0, 128);
  ReadAnalogSensoHandle = osThreadCreate(osThread(ReadAnalogSenso), NULL);

  /* definition and creation of ReadGyroSensor */
  osThreadDef(ReadGyroSensor, Read_Gyro_Ssr, osPriorityLow, 0, 128);
  ReadGyroSensorHandle = osThreadCreate(osThread(ReadGyroSensor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();	// ------------------------------------ uncomment ------------------------

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*
		HAL_I2C_Mem_Read(&hi2c1, (0x68<<1) + 1, 67, 1, data_gyro, 6, 100);
		x_acc = ((int16_t)data_gyro[0] << 8) + data_gyro[1];
		y_acc = ((int16_t)data_gyro[2] << 8) + data_gyro[3];
		z_acc = ((int16_t)data_gyro[4] << 8) + data_gyro[5];
		HAL_Delay(10);
		*/
		
		/*
		if (selCount % 2 == 1) {
			mStop();
			if ((uint32_t)(HAL_GetTick() - ledTime) > 400) {
				ledTime = HAL_GetTick();
				HAL_GPIO_TogglePin(ledPC13_GPIO_Port, ledPC13_Pin);
			}
		} 
		
		else if (selCount % 2 == 0 && selCount > 0) {
			if (modCount == 0)
				mStop();
			else if (modCount == 1)
				mForward();
			else if (modCount == 2)
				mBackward();
			else if (modCount == 3)
				mLeft();
			else if (modCount == 4)
				mRight();
			
			loop_7seg();
		}
		
		modState = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
		selState = HAL_GPIO_ReadPin(SELECT_GPIO_Port, SELECT_Pin);
		
		if (selState == 0 && selStatePre == 0) {
			selStatePre = 1;
			selCount++;
			if (selCount % 2 == 0)
				HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
		} else if (selState == 1) selStatePre = 0;
		
		if(modState == 0 && modStatePre == 0) {
			bzEnable();
			modStatePre = 1;
			if (selCount % 2 == 1)
				modCount++;
			if (modCount >= 5)
				modCount = 0;
				
			Show_7Seg(modCount);
		} else if (modState == 1) modStatePre = 0;
		
		if (bzState == 1 && bzTime < HAL_GetTick()) {
			bzState = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, bzState);	
		}
		*/
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZ_Pin|L7A_Pin|L6B_Pin|L5C_Pin
                          |L4D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L3E_Pin|L2G_Pin|L1F_Pin|EN7LED_Pin
                          |EN7SEG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ledPC13_Pin */
  GPIO_InitStruct.Pin = ledPC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ledPC13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_Pin SELECT_Pin */
  GPIO_InitStruct.Pin = MODE_Pin|SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin L7A_Pin L6B_Pin L5C_Pin
                           L4D_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|L7A_Pin|L6B_Pin|L5C_Pin
                          |L4D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L3E_Pin L2G_Pin L1F_Pin */
  GPIO_InitStruct.Pin = L3E_Pin|L2G_Pin|L1F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN7LED_Pin EN7SEG_Pin */
  GPIO_InitStruct.Pin = EN7LED_Pin|EN7SEG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
void bzEnable(void) {
	bzState = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, bzState);
	bzTime = HAL_GetTick() + 100;
}

void loop_7seg(void) {
	if ((uint32_t)(HAL_GetTick() - led7Time) > 100) {
		led7Time = HAL_GetTick();
		Show_1Seg(counter);
		counter++;
		if (counter >= 7)
			counter = 1;
	}
}

void mRight(void) {
	TIM2 ->CCR1 = 500;
	TIM2 ->CCR2 = 0;	
	TIM4 ->CCR1 = 0;
	TIM4 ->CCR2 = 500;	
}

void mLeft(void) {
	TIM2 ->CCR1 = 0;
	TIM2 ->CCR2 = 500;	
	TIM4 ->CCR1 = 500;
	TIM4 ->CCR2 = 0;	
}

void mStop(void) {
	TIM2 ->CCR1 = 0;
	TIM2 ->CCR2 = 0;	
	TIM4 ->CCR1 = 0;
	TIM4 ->CCR2 = 0;		
}

void mForward(void) {
	TIM2 ->CCR1 = 500;
	TIM2 ->CCR2 = 0;	
	TIM4 ->CCR1 = 500;
	TIM4 ->CCR2 = 0;	
}

void mBackward(void) {
	TIM2 ->CCR1 = 0;
	TIM2 ->CCR2 = 500;	
	TIM4 ->CCR1 = 0;
	TIM4 ->CCR2 = 500;	
}
*/

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Write_IO_PWM */
/**
  * @brief  Function implementing the WriteGPIO_PWM thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Write_IO_PWM */
void Write_IO_PWM(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Read_Analog_Ssr */
/**
* @brief Function implementing the ReadAnalogSenso thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_Analog_Ssr */
void Read_Analog_Ssr(void const * argument)
{
  /* USER CODE BEGIN Read_Analog_Ssr */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Read_Analog_Ssr */
}

/* USER CODE BEGIN Header_Read_Gyro_Ssr */
/**
* @brief Function implementing the ReadGyroSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_Gyro_Ssr */
void Read_Gyro_Ssr(void const * argument)
{
  /* USER CODE BEGIN Read_Gyro_Ssr */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Read_Gyro_Ssr */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
