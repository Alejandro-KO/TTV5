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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VALOR_0 65
#define VALOR_PI 295

#define ENABLE_PIN_q1 GPIO_PIN_1
#define MS0_PIN_q1 GPIO_PIN_9
#define MS1_PIN_q1 GPIO_PIN_7
#define MS2_PIN_q1 GPIO_PIN_5
#define STEP_q1 GPIO_PIN_3
#define DIR_q1 GPIO_PIN_6

#define ENABLE_PIN_q2 GPIO_PIN_4
#define MS0_PIN_q2 GPIO_PIN_2
#define MS1_PIN_q2 GPIO_PIN_0
#define MS2_PIN_q2 GPIO_PIN_11
#define STEP_q2 GPIO_PIN_15
#define DIR_q2 GPIO_PIN_11

#define ENABLE_PIN_q3 GPIO_PIN_9
#define MS0_PIN_q3 GPIO_PIN_9
#define MS1_PIN_q3 GPIO_PIN_7
#define MS2_PIN_q3 GPIO_PIN_15
#define STEP_q3 GPIO_PIN_13
#define DIR_q3 GPIO_PIN_11

#define VELOCIDAD 0.5

#define bufersize 1
#define BUFFER_SIZE 256

#define DEBOUNCE_THRESHOLD 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t byte;
uint8_t buffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;
uint8_t bufferOverflowFlag = 0; // Bandera para indicar desbordamiento

uint8_t stringToSend[] = "Hola";  // Define la cadena que deseas enviar
uint8_t prueba_1[] = "Flag_1";  // Define la cadena que deseas enviar
uint8_t received_data;

char q1[BUFFER_SIZE] = {0};
char q2[BUFFER_SIZE] = {'1','1','2'};
char q3[BUFFER_SIZE] = {0};
char q4[BUFFER_SIZE] = {'1','.','5','7','0','7'};
char q5[BUFFER_SIZE] = {'1','.','5','7','0','7'};

uint32_t radianes_a_valor(float radianes) {
    // Ajusta los radianes negativos a su equivalente positivo en el rango de 0 a 2PI
    if (radianes < 0) {
        radianes += M_PI;
    }

    // Normaliza el valor de radianes en el rango de 0 a PI
    if (radianes > M_PI) {
        radianes = M_PI;
    }

    return VALOR_0 + (uint32_t)((VALOR_PI - VALOR_0) * (radianes / M_PI));
}

uint32_t milimetros_a_pasos(float milimetros) {
    // Calcular el número de pasos necesarios para mover la distancia en milímetros
    float pasos_por_mm = 200.0 / 8.0; // 200 pasos por 8 mm
    return (uint32_t)(fabs(milimetros) * pasos_por_mm);
}

volatile uint8_t FC_Home_q2 = 1;// Variable to control motor state
volatile uint8_t FC_Home_q3 = 1;
volatile uint8_t Paro_emergencia = 1;
volatile uint8_t Contador = 0;

volatile uint8_t button_state = 0; // Estado del botón: 0 (no presionado), 1 (presionado)
volatile uint8_t stable_state = 0;
volatile uint8_t debounce_counter = 0;
volatile uint32_t event_count = 0; // Contador de eventos

volatile int pasos_retroceso = 0;

volatile int bandera = 0;

int paso_actual_q1 = 0;
int paso_actual_q2 = 5350;
int paso_actual_q3 = 0;

float q1_float;
float q4_float;
float q5_float;
int q2_int;
int q3_int;

uint8_t data[1] = "1"; // El dato a transmitir

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void processBuffer(uint8_t *buffer, uint16_t length);
void A4988_q1();
void A4988_q2();
void A4988_q3();
void Home (void);
void Home_q2(void);
void Home_q3(void);
void mover_motorq1_rad(float radianes);
void mover_motorq2_mm(float milimetros);
void mover_motorq3_mm(float milimetros);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart3,&byte,bufersize);

  A4988_q1();
  A4988_q2();
  A4988_q3();
  Home();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(Paro_emergencia == 1)
	{
		q1_float = atof(q1);
		q4_float = atof(q4);
		q5_float = atof(q5);

		// Conversión de q2 y q3 a int (truncando los valores decimales)
		q2_int = (int)atof(q2);
		q3_int = (int)atof(q3);

		mover_motorq1_rad(q1_float);
		HAL_Delay(100);
		mover_motorq2_mm(q2_int);
		HAL_Delay(100);
		mover_motorq3_mm(q3_int);
		HAL_Delay(200);
		TIM2->CCR4 = radianes_a_valor(q4_float); //q4
		HAL_Delay(200);
		TIM2->CCR2 = radianes_a_valor(q5_float); //q5

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // Enciende el LED
		HAL_Delay(1000); // Retardo de 1 segundo (1000 milisegundos)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // Apaga el LED
		HAL_Delay(1000); // Retardo de 1 segundo (1000 milisegundos)

		HAL_UART_Transmit(&huart3, data, 1, 100); // Transmite un byte
		HAL_Delay(1500); // Retardo de 1 segundo (1000 milisegundos)
	}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 15;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 2950;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC7 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE9 PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD13 PD15 PD0
                           PD2 PD4 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_7) {
    	FC_Home_q2 = 0;
    }
    if (GPIO_Pin == GPIO_PIN_9) {
    	FC_Home_q3 = 0;
    }
    if (GPIO_Pin == GPIO_PIN_11) { //rojo
//    	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
//    	HAL_GPIO_WritePin(GPIOE, ENABLE_PIN_q1, GPIO_PIN_SET);
//    	HAL_GPIO_WritePin(GPIOD, ENABLE_PIN_q2, GPIO_PIN_SET);
//    	HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_q3, GPIO_PIN_SET);
    	Paro_emergencia = 0;
    }
    if (GPIO_Pin == GPIO_PIN_13) { //verde
//    	Paro_emergencia = Paro_emergencia + 1;
//    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//    	HAL_GPIO_WritePin(GPIOE, ENABLE_PIN_q1, GPIO_PIN_RESET);
//    	HAL_GPIO_WritePin(GPIOD, ENABLE_PIN_q2, GPIO_PIN_RESET);
//    	HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_q3, GPIO_PIN_RESET);
    	Paro_emergencia = 1;
    	__HAL_TIM_SET_COUNTER(&htim6, 0);
    	HAL_TIM_Base_Start_IT(&htim6);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) { // Asegúrate de que este es el timer correcto
        uint8_t current_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13); // Leer el pin del botón

        if (current_state == stable_state) {
            if (debounce_counter < DEBOUNCE_THRESHOLD) {
                debounce_counter++;
            } else {
                if (button_state != current_state) {
                    button_state = current_state;
                    event_count++; // Incrementar el contador de eventos

                    // Toggle de los LEDs según el contador de eventos
                    if (event_count % 3 == 0) {
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3); // Toggle del LED1
                    }
                    if (event_count % 5 == 0) {
                        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3); // Toggle del LED2
                    }
                }
                // Detener el timer si el estado es estable
                HAL_TIM_Base_Stop_IT(&htim6);
            }
        } else {
            debounce_counter = 0;
        }
        stable_state = current_state;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_LED, GPIO_PIN_SET); // Enciende el LED
        //HAL_UART_Transmit(&huart3,&byte,1, 100); // Envía la cadena a través de UART

        // Almacenar el byte recibido en el buffer si no es '>'
        if (byte != 62) // 62 es el código ASCII para '>'
        {

            if (bufferIndex < BUFFER_SIZE)
            {

                buffer[bufferIndex++] = byte;

            }
            else
            {
                // Manejar el caso de desbordamiento del buffer
                bufferOverflowFlag = 1; // Establecer la bandera de desbordamiento
                bufferIndex = 0; // Opcional: restablecer el índice del buffer
            }
        }
        else
        {
            // Aquí puedes manejar el caso cuando se recibe '>'
            // Por ejemplo, procesar el buffer y restablecer bufferIndex
        	 //HAL_UART_Transmit(&huart1, prueba_1, sizeof(prueba_1) - 1, 100);
        	 //HAL_UART_Transmit(&huart1, buffer,bufferIndex, 100);// Envía la cadena a través de UART
            processBuffer(buffer, bufferIndex);
            bufferIndex = 0;
        }

        HAL_UART_Receive_IT(&huart3, &byte, 1);


        // Vuelve a habilitar la recepción por interrupción

    }
}

void processBuffer(uint8_t *buffer, uint16_t length)
{
    if (bufferOverflowFlag)
    {
        // Manejar el desbordamiento del buffer
        // Por ejemplo, enviar un mensaje de error o realizar acciones correctivas
        HAL_UART_Transmit(&huart3, (uint8_t *)"Buffer overflow\n", 16, 100);
        bufferOverflowFlag = 0; // Restablecer la bandera de desbordamiento
        return;
    }

    // Variables para almacenar las partes separadas
//    char q1[BUFFER_SIZE] = {0};
//    char q2[BUFFER_SIZE] = {0};
//    char q3[BUFFER_SIZE] = {0};
//    char q4[BUFFER_SIZE] = {0};

    // Punteros para la división de la cadena
    char *ptr = (char *)buffer;
    char *start = ptr;
    char *end = strchr(start, 'a');

    if (end != NULL)
    {
        strncpy(q1, start, end - start);
        start = end + 1;
        end = strchr(start, 'b');

        if (end != NULL)
        {
            strncpy(q2, start, end - start);
            start = end + 1;
            end = strchr(start, 'c');

            if (end != NULL)
            {
                strncpy(q3, start, end - start);
                start = end + 1;
                end = strchr(start, 'd');

                if(end != NULL){
                	 strncpy(q4, start, end - start);
                	 start = end + 1;
                	 strcpy(q5, start);
                }


            }
        }
    }



    // Enviar cada parte a través de UART para verificar
    //HAL_UART_Transmit(&huart3, (uint8_t *)q1, strlen(q1), 100); // 0 puntos desfazados
    //HAL_UART_Transmit(&huart1, (uint8_t *)q2, strlen(q2), 100); // 5 puntos desfazados
    //HAL_UART_Transmit(&huart1, (uint8_t *)q3, strlen(q3), 100); // 2 puntos malos
    //HAL_UART_Transmit(&huart1, (uint8_t *)q4, strlen(q4), 100); // Enviar q4 si hay datos

}

void A4988_q1(){
	HAL_GPIO_WritePin(GPIOE, ENABLE_PIN_q1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MS0_PIN_q1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MS1_PIN_q1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MS2_PIN_q1, GPIO_PIN_RESET);
}

void A4988_q2(){
	HAL_GPIO_WritePin(GPIOD, ENABLE_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS0_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS1_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS2_PIN_q2, GPIO_PIN_RESET);
}

void A4988_q3(){
	HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS0_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS1_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS2_PIN_q3, GPIO_PIN_RESET);
}

void Home (void){
	Home_q2();
	Home_q3();
	TIM2->CCR2 = radianes_a_valor(M_PI/2); //q5
	TIM2->CCR4 = radianes_a_valor(M_PI/2); //q4
}

void Home_q2(void){
	while(FC_Home_q2){
		HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_RESET);  //Retroceso
		for (int i = 0; i < 100000 && FC_Home_q2; i++) {
			HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
			HAL_Delay(VELOCIDAD);
			HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
			HAL_Delay(VELOCIDAD);
		}
		if (!FC_Home_q2) break;
		HAL_Delay(500);
	}

	HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_SET); //Avance
	for (int i = 0; i < 2500; i++) {
		HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
		HAL_Delay(VELOCIDAD);
		HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
		HAL_Delay(VELOCIDAD);
		paso_actual_q2--;
	}
	HAL_Delay(500);
	FC_Home_q2 = 1;
}

void Home_q3(void){
	while(FC_Home_q3){
		HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_RESET);  //Abajo
		for (int i = 0; i < 100000 && FC_Home_q3; i++) {
			HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
			HAL_Delay(VELOCIDAD);
			HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
			HAL_Delay(VELOCIDAD);
		}
		if (!FC_Home_q3) break;
		HAL_Delay(500);
	}

	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_SET); //Arriba
	for (int i = 0; i < 80; i++) {
		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
		HAL_Delay(VELOCIDAD);
		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
		HAL_Delay(VELOCIDAD);
	}
	HAL_Delay(500);
	FC_Home_q3 = 1;
}

void mover_motorq1_rad(float radianes){

    int pasos = (int)((radianes / (2 * M_PI)) * 400);
    int nuevo_paso = pasos;
    int diferencia_pasos = nuevo_paso - paso_actual_q1;

    if (diferencia_pasos > 0) {
        // Movimiento hacia adelante
    	HAL_GPIO_WritePin(GPIOD, DIR_q1, GPIO_PIN_RESET); //Antihorario
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

//    if(radianes == (2*M_PI))
//    {
//    	radianes = 0;
//    }

    else if (diferencia_pasos < 0) {
        // Movimiento hacia atrás
    	HAL_GPIO_WritePin(GPIOD, DIR_q1, GPIO_PIN_SET); //Horario
    	diferencia_pasos = -diferencia_pasos;
    	for (int i = 0; i < diferencia_pasos ; i++) {
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    paso_actual_q1 = nuevo_paso;
    HAL_Delay(500);
}

void mover_motorq2_mm(float milimetros){

	//milimetros = milimetros - 500;

    if (milimetros < 0) {
        milimetros = 0;
    }
    else if (milimetros > 210) {
        milimetros = 210;
    }

    uint32_t pasos = milimetros_a_pasos(milimetros);
    int diferencia_pasos = pasos - paso_actual_q2;

    if (diferencia_pasos != 0) {
        if (diferencia_pasos > 0) {
        	 HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_RESET); //Retroceso
        }
        else {
        	HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_SET); //Avance
            diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
        }

        for (int i = 0; i < diferencia_pasos; i++) {
        	HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
        	HAL_Delay(VELOCIDAD);
        	HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
        	HAL_Delay(VELOCIDAD);
        }

        paso_actual_q2 = pasos;
    }

    HAL_Delay(500);
}

void mover_motorq3_mm(float milimetros){

	if (milimetros < 0) {
		milimetros = 0;
	}
	else if (milimetros > 215) {
		milimetros = 215 ;
	}

    uint32_t pasos = milimetros_a_pasos(milimetros);
    int nuevo_paso = pasos;
    int diferencia_pasos = nuevo_paso - paso_actual_q3;

    if (diferencia_pasos > 0) {
    	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_SET); //Arriba
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    else if (diferencia_pasos < 0) {
    	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_RESET);  //Abajo
    	diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    paso_actual_q3 = nuevo_paso;
    HAL_Delay(500);
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
