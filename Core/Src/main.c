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
  *
  * Code for a 2-channel (PA0, PA1) voltmeter using an STM32G030F6P6 cpu's ADC.
  *
  * The measurement (in volts) is displayed on a SSD1306-based I2C display.
  * Due to the small size of the flash memory (32K), floating point printf
  * formatting code results in the code not fitting in flash, so we calculate
  * the integer and fractional part separately and print them with a dot between
  * them.
  *
  * In order to measure voltages higher than the 3.3V reference, each channel
  * the board has a voltage divider (51K/(51K + 200K)) enabled by a MOSFET that
  * is controlled by a GPIO output pin (A2 controls channel A0, A3 controls
  * channel A1).  With the voltage divider, each channel can measure up to
  * around 16V (3.3 * (200 + 51)/51).
  * When the MOSFET is off, the ADC measures the voltage directly (through the
  * 200K resistor).  This allows us to the full scale of the ADC for voltages
  * lower than around 3.3V.
  *
  * To further increase the resolution of the ADC, each channel is 16x oversampled
  * to give a full scale of 12 bits + 4 bits = 16 bits.
  *
  * The ADCs are continually sampled by DMA into a memory buffer.
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include <stdio.h>

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

/* USER CODE BEGIN PV */

uint16_t adcvals[2];
uint8_t fi2cadr = 0;
// We use millivolts instead of a floating point number for volts
// because including code to support printing floating point numbers
// exceeds the flash size.
uint16_t millivolts[2];
float scale[2]; // Scaling factor based on the status of the PA2, PA3 pins
GPIO_PinState pinstate[2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// display data on SSD1306 screen
// _BIG_DISP is used for 0.96" 128x64 screen
// otherwise a 0.49" 64x32 screen is used
// Note that we don't use %f for the sprintf because enabling floating point
// formats in the code causes the code size to increase beyond the flash size.
//#define _BIG_DISP
#ifdef _BIG_DISP
void display() {
	char buf[20];
	uint8_t rpix = 0, rht = 12;
	uint16_t volts, mvolts;

	 // row 0
	 ssd1306_SetCursor(0, rpix);
	 sprintf(buf, "%6d %6d", adcvals[0], adcvals[1]);
	 ssd1306_WriteString(buf, Font_7x10, White);

	 rpix += rht; // row 1
	 rht = 20;

	 ssd1306_SetCursor(0, rpix);
	 volts = millivolts[0] / 1000;
	 mvolts = millivolts[0] % 1000;
	 sprintf(buf, "0:%2u.%03u", volts, mvolts);
	 ssd1306_WriteString(buf , Font_11x18, White);

	 rpix += rht; // row 2

	 ssd1306_SetCursor(0, rpix);
	 volts = millivolts[1] / 1000;
	 mvolts = millivolts[1] % 1000;
	 sprintf(buf, "1:%2u.%03u", volts, mvolts);
	 ssd1306_WriteString(buf , Font_11x18, White);

	 ssd1306_UpdateScreen(&hi2c1);
}
#else
void display() {
	char buf[20];
	uint8_t rpix = 32, rht = 12;
	uint16_t volts, mvolts;

	 // row 0
	 ssd1306_SetCursor(32, rpix);
	 volts = millivolts[0] / 1000;
	 mvolts = millivolts[0] % 1000;
	 sprintf(buf, "0:%2u.%03u", volts, mvolts);
	 ssd1306_WriteString(buf , Font_7x10, White);

	 rpix += rht;

	 // row 1
	 ssd1306_SetCursor(32, rpix);
	 volts = millivolts[1] / 1000;
	 mvolts = millivolts[1] % 1000;
	 sprintf(buf, "1:%2u.%03u", volts, mvolts);
	 ssd1306_WriteString(buf , Font_7x10, White);

	 ssd1306_UpdateScreen(&hi2c1);
}
#endif // _BIG_DISP

void setScale(int chan, GPIO_PinState st) {
	pinstate[chan] = st;
	if (chan == 0) {
		HAL_GPIO_WritePin(PA2OUT_GPIO_Port, PA2OUT_Pin, st);
	} else {
		HAL_GPIO_WritePin(PA3OUT_GPIO_Port, PA3OUT_Pin, st);
	}
	if (st == GPIO_PIN_RESET) {
		scale[chan] = 0.050354; // 3300/65536;
	} else {
		scale[chan] = 0.247821; // ((200 + 51)/51) * 3300/65536;
	}
}

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  for (int i2a = 1; i2a < 127; i2a++) {
  	if (HAL_I2C_IsDeviceReady(&hi2c1, i2a << 1, 1, 10) == HAL_OK) {
  		fi2cadr = i2a;
  	}
  }

  if (ssd1306_Init(&hi2c1) != 0) {
    Error_Handler();
  }

  HAL_Delay(100);

  ssd1306_Fill(Black);
  ssd1306_UpdateScreen(&hi2c1);

  HAL_Delay(100);

  setScale(0, GPIO_PIN_RESET);
  setScale(1, GPIO_PIN_RESET);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcvals, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for (int i = 0; i < 2; i++) {
		  if ((adcvals[i] > 65000) && (pinstate[i] == GPIO_PIN_RESET)) {
			  setScale(i, GPIO_PIN_SET);
		  } else if ((adcvals[i] < 13000) && (pinstate[i] == GPIO_PIN_SET)) {
			  setScale(i, GPIO_PIN_RESET);
		  }
	  }
	  HAL_Delay(100);
	  for (int i = 0; i < 2; i++) {
		  millivolts[i] = adcvals[i] * scale[i];
	  }
	  display();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
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
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA2OUT_Pin|PA3OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA2OUT_Pin PA3OUT_Pin */
  GPIO_InitStruct.Pin = PA2OUT_Pin|PA3OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
