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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define true 1
#define false 0
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE 256 //Hz
#define ADC_BUF_LEN 2048
#define sampleForSNR 2000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t pulse = 0; //lowest part of crest
float bloodOx = 0;
char pulseBuf[10];
//EMA Filter Setup
//0.08 without analog RC LP filter
//0.06 with analog rc lp filter
float alpha = 0.06f;
float prev_RedVal = 0.0;
float prev_IRVal = 0.0;

struct timeValue
{
	uint32_t value;
	uint32_t timestamp;
};
struct timeValue lastRead, lastLastRead;

uint32_t lastPeak = 0;
struct timeValue redMax, redMin;
struct timeValue irMax, irMin;
uint32_t tick;
int peakRange = 50;
int RedLEDActive = true;
int counter = 0;

uint64_t signal[sampleForSNR] = {0};
uint16_t AdcVal = 0;
int isPeak = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//void pulseCalc(uint32_t curVal, uint32_t tick);
void transmitData();
void updateMaxMin(struct timeValue *max, struct timeValue *min, float prev_val);
void peakFinder();
void calculateBloodOx();
float ratioToBloodOx(float ratio);
float32_t map(float32_t prev_val, int inMax, int inMin, int outMax, int outMin);
int subtractArray(int start, int subtractVal, int size);
void ProcessSignalToNoise();
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2); // Start Timer3 (Trigger Source For ADC1)
  HAL_ADC_Start_IT(&hadc1); // Start ADC Conversion
  HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, IR_LED_Pin, GPIO_PIN_RESET);
//  HAL_TIM_Base_Start_IT(&htim3);
//  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  int delayTime = 17;
//  	int delayTime = 250;
	HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, IR_LED_Pin, GPIO_PIN_RESET);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 88;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 42967;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IR_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = IR_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	tick = HAL_GetTick();
	AdcVal = HAL_ADC_GetValue(&hadc1) * 1;
	alpha = 0.06;
	//EMA Filter
	prev_RedVal = alpha * (float32_t)AdcVal + (1.0f-alpha) * prev_RedVal; //filtered
	updateMaxMin(&redMax, &redMin, prev_RedVal);
	peakFinder();
	transmitData();

	//SNR calculations
//	if (counter < 1000) counter++;
//	else if (counter >= 1000 && counter < 1000 + sampleForSNR)
//	{
////		signal[counter - 1000] = prev_RedVal;
//				signal[counter - 1000] = AdcVal;
//
//		counter++;
//		transmitData();
//	} else ProcessSignalToNoise();

}
void ProcessSignalToNoise()
{
	float sum_signal = 0;
	for (int i = 0; i < sampleForSNR; i++)
	{
		sum_signal += signal[i];
	}
	float average = sum_signal / (float)sampleForSNR;
	float power = 0;
	//subtract mean aka dc offset aka average
	//determine RMS
	for (int j = 0; j < sampleForSNR; j++)
	{
		power += (signal[j] - average) * (signal[j] - average);
	}
	power /= (float)sampleForSNR;
	//transmit power
	char buffer[10];
	snprintf(buffer, sizeof(buffer), "%u\r\n", (int)power);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	while (1)
	{
		//stop indefinitely
	}
}
/**************************************************************
 * Function: updateMaxMin(struct timeValue max, struct timeValue min, float prev_val)
 * Inputs: Max, Min, Current Value
 * Outputs: None
 * Update min/max for Red and IR readings in the last 4 seconds
 **************************************************************/
void updateMaxMin(struct timeValue *max, struct timeValue *min, float prev_val)
{
	if (prev_val > max->value || tick > max->timestamp + 1000 * 4) {max->value = prev_val; max->timestamp = tick;}
	else if (prev_val < min->value || tick > min->timestamp + 1000 * 4) {min->value = prev_val; min->timestamp = tick;}
	if (max->value - min->value > (redMax.value - redMin.value - 1) && tick > 10)
	{
		//update threshhold window
		if (max->timestamp > min->timestamp)
		{
			min->value = max->value - (redMax.value - redMin.value - 1);
			min->timestamp = max->timestamp - 1;
		}
		else
		{
			max->value = min->value + (redMax.value - redMin.value - 1);
			max->timestamp = min->timestamp - 1;
		}
	}
}

/**************************************************************
 * Function: peakFinder()
 * Inputs/Outputs: None
 * Find peak if Red LED value is within 8 arbitrary units of min
 * 0.5 seconds between peaks at minimum, and value needs to be gte
 * 	than surrounding ones
 **************************************************************/
void peakFinder()
{
	if (lastRead.value > redMax.value - redMin.value - 1
			&& lastRead.value >= prev_RedVal
			&& lastRead.value >= lastLastRead.value
			&& lastRead.timestamp - lastPeak > 500)
	{
		//is peak
		HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
		pulse = (60.0f * 1000.0f / ((float)lastRead.timestamp - (float)lastPeak));
		lastPeak = lastRead.timestamp;
		isPeak = true;
	}
	isPeak = false;
	//shift all vals back
	lastLastRead = lastRead;
	lastRead.value = prev_RedVal;
	lastRead.timestamp = tick;
}

void calculateBloodOx()
{
	float numerator = prev_RedVal / redMin.value;
	float denom = prev_IRVal / irMin.value;
	bloodOx = ratioToBloodOx(numerator / denom);
}

//apply piecewise ratio to blood ox per Beer-Lambert Law
float ratioToBloodOx(float ratio)
{
	if (ratio >= 0.4f && ratio <= 1)
		return -25.0f * ratio + 100.0;
	else
		return -32.6923f * ratio + 85;
}
/**************************************************************
 * Function: transmitData()
 * Inputs/Outputs: None
 * Transmit data in form: time, redLED, IRLED, Pulse, Blood Ox
 **************************************************************/
void transmitData()
{
	//transmit time for analysis
	char timeBuf[10];
	snprintf(timeBuf, sizeof(timeBuf), "%lu,", tick);
	HAL_UART_Transmit(&huart2, (uint8_t*)timeBuf, strlen(timeBuf), HAL_MAX_DELAY);

	//transmit red LED data
	char buffer[10];
	snprintf(buffer, sizeof(buffer), "%u,", (int)prev_RedVal);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

//	//transmit original data
//	snprintf(buffer, sizeof(buffer), "%u\r\n", (int)AdcVal);
//	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//	//transmit IR LED data
//	snprintf(buffer, sizeof(buffer), "%u,", (int)prev_IRVal);
//	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
	//pulse
	snprintf(pulseBuf, sizeof(pulseBuf), "%u,", pulse);
	HAL_UART_Transmit(&huart2, (uint8_t*)pulseBuf, strlen(pulseBuf), HAL_MAX_DELAY);

	snprintf(buffer, sizeof(buffer), "%u\r\n", isPeak);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//
//	//transmit Blood Ox data
//	snprintf(buffer, sizeof(buffer), "%u\r\n", (int)bloodOx);
//	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
float32_t map(float32_t prev_val, int inMax, int inMin, int outMax, int outMin)
{
	return (prev_val - (float)inMin) * ((outMax-outMin) / (inMax - inMin)) + outMin;
}
int subtractArray(int start, int subtractVal, int size)
{
	return (start - subtractVal >= 0) ? start - subtractVal : size + start - subtractVal;
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
