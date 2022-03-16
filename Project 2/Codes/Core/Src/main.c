/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * Extract the values needed from the memory
 * Memory locations provided in Datasheet on P44
 */
#define TS_CAL1 ((uint16_t*)((uint32_t) 0x1FFF75A8))
#define TS_CAL2 ((uint16_t*)((uint32_t) 0x1FFF75CA))
#define VREFINT ((uint16_t*)((uint32_t) 0x1FFF75AA))

/**
 * Temperature raw data acquirements
 * Values provided in Datasheet on P44
 */
#define TS_CAL1_TEMP 30
#define TS_CAL2_TEMP 130

// This global variable to identify which channel are we currently in by switching the value of flag,
// we can change the channel
#define __VREFFANALOG_VOLTAGE__ 3000	// ADC reference voltage in mV


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
int flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Overwrite the write function for ITM
 */
int _write(int file, char *ptr, int len){
	int i = 0;

	for (i = 0; i < len; i++){
		ITM_SendChar((*ptr++));
	}

	return len;
}


/**
  * @brief Convert the value obtained by the Vref channel to the actual reference voltage
  * @note The formula used is provided in Chip Document on P691
  * @note Vdda is provided in Datasheet on P44
  * @param raw_data: The value obtained by the Vref channel
  * @retval int: The actual reference voltage
  */
int convert_VREFINT(uint32_t raw_data){
	return 3000.0f * (*VREFINT) / raw_data;

}


/**
  * @brief Convert the value obtained by the temperature channel to the actual temperature
  * @note The formula used is provided in Chip Document on P689
  * @note Values used is provided in Datasheet on P44
  * @param raw_data: The value obtained by the temperature channel
  * @parm factor: The factor needed to times with the raw data since VREF+ might be different from the TS_CAL1/TS_CAL2 values
  * @retval int: The actual temperature
  */
int convert_TEMPERATURE(uint32_t raw_data, float factor){
	int temperature;

	temperature = (int)(TS_CAL2_TEMP - TS_CAL1_TEMP) / ((float)(*TS_CAL2) - (float)(*TS_CAL1))
			* ((float)raw_data *factor  - (float)(*TS_CAL1)) + 30;

	return temperature;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char status = 0;
	uint32_t raw_data;
	int converted_data;

	/**
	 * Flag is used to represent the channel currently used
	 * 1 for Vref and 0 for temperature
	 * Set the flag to be 1 as default, since Vref is needed for the calculation of internal temperature
	 */
	flag = 1;

	/**
	 * This factor is used to adjust the conversion of internal temperature,
	 * since the actual reference voltage is different from the default voltage,
	 * so the data obtained by TS need to multiply this factor.
	 * From Chip Document on P690
	 */
	float factor = 0;

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	char *msg;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Read the button pin
	status = HAL_GPIO_ReadPin(my_button_GPIO_Port, my_button_Pin);

	if (status == 0){
		// When the button is pressed, toggle the LED
		HAL_GPIO_TogglePin(my_LED_GPIO_Port, my_LED_Pin);

		/**
		 * Toggle the flag
		 * When a button is pressed, it indicates that the channel need to be switched
		 * This will be done by toggling the flag and is implemented in function MX_ADC1_Init at line 347
		 */
		if (flag == 1){
			flag = 0;

		}else if(flag == 0){
			flag = 1;
		}
	}

	MX_ADC1_Init();

	// Start ADC
	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}

	// Get ADC value
	raw_data = HAL_ADC_GetValue(&hadc1);

	/**
	 * If the flag is 1, it means that we are measuring the reference voltage
	 * Therefore, convert the value to voltage and print it
	 */
	if (flag == 1){
		msg = "Voltage: ";
		converted_data = convert_VREFINT(raw_data);
		printf("%s%d mV\n", msg, converted_data);

		factor = (float)converted_data / __VREFFANALOG_VOLTAGE__;
	}

	/**
	 * If the flag is 0, it means that we are measuring the internal temperature
	 * Therefore, convert the value to temperature and print it
	 */
	else if(flag == 0){
		msg = "Temperature: ";
		converted_data = convert_TEMPERATURE(raw_data, factor);
		printf("%s%d degree Celsius\n", msg, converted_data);
	}


	/**
	 * Checked with the professor, this step is used to save power
	 * When using it, the clock time will be extended
	 * To save development time, this step is not implemented
	 */
	// HAL_ADC_Stop(&hadc1);


	/**
	 * 0.2s is delayed after each reading
	 * This time is implemented to make the readings be discrete
	 * And for the user to complete pushing the button
	 */
	HAL_Delay(200);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /**
   * The flag will decide which channel we want to use
   * the sampling time is calculated by the formula from Datasheet P183
   * tconv = ts + 12.5 cycles for successive approximation = 15 to 653
   * ts_vrefin = 4us from Datasheet P122
   * ts_temp = 5us from Datasheet P207
   */
  if(flag == 0){
	  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	  sConfig.SamplingTime =ADC_SAMPLETIME_247CYCLES_5;

  }else{
	  sConfig.Channel = ADC_CHANNEL_VREFINT;
	  sConfig.SamplingTime =ADC_SAMPLETIME_247CYCLES_5;
  }

  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(my_LED_GPIO_Port, my_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : my_button_Pin */
  GPIO_InitStruct.Pin = my_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(my_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : my_LED_Pin */
  GPIO_InitStruct.Pin = my_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(my_LED_GPIO_Port, &GPIO_InitStruct);

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

