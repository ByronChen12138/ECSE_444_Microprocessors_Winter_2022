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
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))

/**
 * @brief: The type kalman_state which contains q, r, x, p, k
 */
typedef struct kalman_states{
	float q, r, x, p, k;
}kalman_state;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN(n, m) ((n < m) ? n : m)
#define MAX(n, m) ((n < m) ? m : n)
extern void kalman_asm(float *array, float measurement);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */

/**
 * @brief: Initialize a kalman_state struct
 * @param kalman_state: kalman_state struct needed to be initialized
 * @param q: process noise covariance
 * @param r: measurement noise covariance
 * @param x: estimated value
 * @param p: estimation error covariance
 * @param k: adaptive Kalman filter gain
 * @retval: kalman_state initialized
 */
kalman_state *create_kalman_state(float q, float r, float x, float p, float k){
	kalman_state *new_state = malloc(sizeof(struct kalman_states));

	new_state->q = q;
	new_state->r = r;
	new_state->x = x;
	new_state->p = p;
	new_state->k = k;

	return new_state;
}

/**
 * @brief: Test if the number is overflow
 * @param num_to_test: The float number to be tested
 * @retval: Return -1 if not overflow; otherwise, return 0
 */
int is_overflow(float num_to_test){
	if(num_to_test <= FLT_MAX){
		return -1;
	}

	return 0;
}

/**
 * @brief: Update the kalman state
 * @param kalman_state: kalman_state to be updated
 * @param measurement: the measurement
 * @retval: Return -1 if failed to calculate, 0 if success
 */
int update(kalman_state *kstate, float measurement){

	kstate->p = kstate->p + kstate->q;
	if(is_overflow(kstate->p) == 0) return -1;

	if(kstate->p + kstate->r == 0) return -1;
	kstate->k = kstate->p / (kstate->p + kstate->r);
	if(is_overflow(kstate->k) == 0) return -1;

	kstate->x = kstate->x + kstate->k * (measurement - kstate->x);
	if(is_overflow(kstate->x) == 0) return -1;


	kstate->p = (1 - kstate->k) * kstate->p;
	if(is_overflow(kstate->p) == 0) return -1;

	return 0;
}

/**
 * @brief: A function to process the filter data in asm code
 * @param InputArray: The array of measurements
 * @param OutputArray: The array of values x obtained by Kalman fiter calculations over the input field
 * @param kstate: The kstate struct contains the Kalman filter state that is readily available for debugging
 * @param Length: Specifies the length of the data array to process
 * @retval: return 0 if the function executed properly; otherwise, return 1
 */
int Kalmanfilter_asm(float *InputArray, float *OutputArray, kalman_state *kstate, int Length){
	float inputs[6] = {kstate->q, kstate->r, kstate->x, kstate->p, kstate->k, 0};

	  for(int i = 0; i < Length; i++){
		  kalman_asm(inputs, InputArray[i]);
		  if (inputs[5] == -1){
			  printf("Stop due to error!");
			  return -1;
		  }
		  OutputArray[i] = inputs[2];
	  }
	  return 0;
}

/**
 * @brief: A function to process the filter data in C code
 * @param InputArray: The array of measurements
 * @param OutputArray: The array of values x obtained by Kalman fiter calculations over the input field
 * @param kstate: The kstate struct contains the Kalman filter state that is readily available for debugging
 * @param Length: Specifies the length of the data array to process
 * @retval: return 0 if the function executed properly; otherwise, return 1
 */
int Kalmanfilter_C(float *InputArray, float *OutputArray, kalman_state *kstate, int Length){
	int result;

	for(int i = 0; i < Length; i++){
		result = update(kstate, InputArray[i]);
		if (result == -1){
			printf("Stop due to error!");
			return -1;
		}
	 OutputArray[i] = kstate->x;
	}

	return 0;
}


/**
 * @brief: Calculte the differences of two arrays
 * @param differences: The output array with calculated differences
 * @param data_obtained: The data obtained by Kalman filter tracking
 * @param original_data: The original data
 * @param Length: Specifies the length of the data array to process
 */
void get_differences(float *differences, float *data_obtained, float *original_data, int Length){
	for(int i = 0; i < Length; i++){
		differences[i] = fabsf(original_data[i] - data_obtained[i]);
	}
}


/**
 * @brief: Calculte the average
 * @param differences: The array with the differences to be calculated
 * @param Length: Specifies the length of the data array to process
 * @retval: return the average
 */
float average(float *differences, int Length){
	float sum = 0;

	for(int i = 0; i < Length; i++){
		sum += differences[i];
	}

	return sum / Length;

}

/**
 * @brief: Calculte the Standard Deviation with the given array
 * @param differences: The array with the differences to be calculated
 * @param average: The average of the differences
 * @param Length: Specifies the length of the data array to process
 * @retval: return the average
 */
float standard_deviation(float *differences, float average, int Length){
	float sum = 0;

	for(int i = 0; i < Length; i++){
		float diff = differences[i] - average;
		diff = powf(diff, 2);
		sum += diff;
	}

	sum = sum / Length;
	return sqrtf(sum);
}

/**
 * @brief: Calculte the Correlation with the given array
 * @param measurements: The input measurements
 * @param OutputArray: x's after each proces
 * @param Length: The length of the array
 * @param correlation: The array of the answer
 * @retval: None
 */
void correlation(float *measurements, float *OutputArray, int Length, float *correlation){
	float length = 2 * Length - 1;

	for(int i = 0; i < length; i++){
		float sum = 0;
		int start_index = 0;
		int start_index_B = Length - i - 1;

		for(start_index; start_index <= i; start_index++){
			if(start_index > Length - 1){
				sum += 0;
				start_index_B++;

			}else if(start_index_B > Length - 1){
				sum += 0;
				start_index_B++;

			}else if(start_index_B < 0){
				sum += 0;
				start_index_B ++;

			}else{
			sum += measurements[start_index] * OutputArray[start_index_B];
			start_index_B++;
			}
		}
		correlation[i] = sum;
	}
}

/**
 * @brief: Calculte the Convolution with the given array
 * @param measurements: The input measurements
 * @param OutputArray: x's after each proces
 * @param Length: The length of the array
 * @param convolution: The array of the answer
 * @retval: None
 */
void convolution(float *measurements, float *OutputArray, int Length, float *convolution){
	float length = 2 * Length - 1;

		for(int i = 0; i < length; i++){
			float sum = 0;
			int start_index = i;
			int start_index_B = 0;

			for(start_index; start_index >= 0; start_index--){
				if(start_index > Length - 1){
					sum += 0;
					start_index_B++;

				}else if(start_index_B > Length - 1){
					break;

				}else{
				sum += measurements[start_index] * OutputArray[start_index_B];
				start_index_B++;
				}
			}
			convolution[i] = sum;
		}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DELTA           (0.000001f)
#define MAX_BLOCKSIZE   32
#define LENGTH          101
/* ----------------------------------------------------------------------
* Declare Global variables
* ------------------------------------------------------------------- */
arm_status status;       /* Status*/

/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	float measurements[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
						10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
						9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
						9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
						10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
						10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
						10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
						10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
						9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
						10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
						10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
						10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
						10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
						10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
						9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
						10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
						9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
						10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
						9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
						10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
						9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
						10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
						9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
						9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
						10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
						9.5799256668};

	// The inputs for the Kalman Filter 0.1, 0.1, 0.1, 5, 0
	kalman_state *kstate_asm = create_kalman_state(0.1, 0.1, 5, 0.1, 0);
	kalman_state *kstate_c = create_kalman_state(0.1, 0.1, 5, 0.1, 0);
	int result_asm;
	int result_c;

//	//Assign the measurements
//	float measurements[LENGTH];
//	for(int i = 0; i < LENGTH; i++){
//		measurements[i] = i;
//	}

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

	ITM_Port32(31) = 1;

	// Run the kalman subroutine in asm repeatedly
	float output_asm[LENGTH];
	result_asm = Kalmanfilter_asm(measurements, output_asm, kstate_asm, LENGTH);

	ITM_Port32(31) = 2;

	// Run the kalman subroutine in C repeatedly
	float output_c[LENGTH];
	result_c = Kalmanfilter_C(measurements, output_c, kstate_c, LENGTH);

	ITM_Port32(31) = 3;

	/* ----------------------------------------------------------------------
	** Dealing with the failures
	** ------------------------------------------------------------------- */
	if (result_asm == -1){
		status = ARM_MATH_TEST_FAILURE;
	}

	if (result_c == -1){
			status = ARM_MATH_TEST_FAILURE;
		}

	float difference_asm_c[LENGTH];
	get_differences(difference_asm_c, output_asm, output_c, LENGTH);

	/* ----------------------------------------------------------------------
	** Data Processing
	** ----------------------------------------------------------------------
	** Difference
	** ------------------------------------------------------------------- */

	ITM_Port32(31) = 4;

	// Get the difference between measurements and output x's
	float difference[LENGTH];
	get_differences(difference, measurements, output_c, LENGTH);

	ITM_Port32(31) = 5;

	// Get the differences in CMSIS
	float32_t difference_CMSIS[LENGTH];
	uint32_t length_CMSIS = LENGTH;
	arm_sub_f32(output_c, measurements, difference_CMSIS, length_CMSIS);

	ITM_Port32(31) = 6;

	for(int i = 0; i < length_CMSIS; i++){
		difference_CMSIS[i] = fabsf(difference_CMSIS[i]);
	}

	/* ----------------------------------------------------------------------
	** Average
	** ------------------------------------------------------------------- */

	ITM_Port32(31) = 7;

	// Get the mean of the differences
	float mean = average(difference, LENGTH);

	ITM_Port32(31) = 8;

	// Get the average with CMSIS
	float32_t avg_CMSIS[1];
	arm_mean_f32(difference_CMSIS, length_CMSIS, avg_CMSIS);

	/* ----------------------------------------------------------------------
	** Standard Deviation
	** ------------------------------------------------------------------- */

	ITM_Port32(31) = 9;

	// Get the standard deviation of the differences
	float SD = standard_deviation(difference, mean, LENGTH);

	ITM_Port32(31) = 10;

	// Get the standard deviation with CMSIS
	float32_t SD_CMSIS[1];
	arm_std_f32(difference_CMSIS, length_CMSIS, SD_CMSIS);

	/* ----------------------------------------------------------------------
	** Correlation
	** ------------------------------------------------------------------- */

	ITM_Port32(31) = 11;

	// Get the correlation between the measurements and the output x's
	float correlation_c[2 * LENGTH - 1];
	correlation(measurements, output_c, LENGTH, correlation_c);

	ITM_Port32(31) = 12;

	// Get the correlation of input and output vectors
	float32_t correlation_CMSIS[2 * LENGTH - 1];
	arm_correlate_f32(measurements, length_CMSIS, output_c, length_CMSIS, correlation_CMSIS);



	/* ----------------------------------------------------------------------
	** Convolution
	** ------------------------------------------------------------------- */

	ITM_Port32(31) = 13;

	// Get the convolution between the measurements and the output x's
	float convolution_c[2 * LENGTH - 1];
	convolution(measurements, output_c, LENGTH, convolution_c);

	ITM_Port32(31) = 14;

	// Get the convolution of input and output vectors
	float32_t convolution_CMSIS[2 * LENGTH - 1];
	arm_conv_f32(measurements, length_CMSIS, output_c, length_CMSIS, convolution_CMSIS);

	ITM_Port32(31) = 15;

	/* ----------------------------------------------------------------------
	** Comparision CMSIS and C
	** ------------------------------------------------------------------- */
	float diff_CMSIS_c_sub[LENGTH];
	get_differences(difference, difference_CMSIS, diff_CMSIS_c_sub, LENGTH);
	float avg_diff_CMSIS_c_sub =  average(diff_CMSIS_c_sub, LENGTH);

	float diff_CMSIS_c_avg = fabsf(avg_CMSIS[0] - mean);

	float diff_CMSIS_c_std = fabsf(SD_CMSIS[0] - SD);

	float difference_correlation[2 * LENGTH - 1];
	get_differences(difference_correlation, correlation_c, correlation_CMSIS, 2 * LENGTH - 1);
	float avg_difference_correlation =  average(difference_correlation, 2 * LENGTH - 1);

	float difference_convolution[2 * LENGTH - 1];
	get_differences(difference_convolution, convolution_c, convolution_CMSIS, 2 * LENGTH - 1);
	float avg_difference_convolution =  average(difference_convolution, 2 * LENGTH - 1);


//	/* Comparison of dot product value with reference */
//	if(diff > DELTA){
//		status = ARM_MATH_TEST_FAILURE;
//	}

	/* ----------------------------------------------------------------------
	** Exception Testers
	** ------------------------------------------------------------------- */
	float overflow_output_asm[LENGTH];
	float overflow_output_c[LENGTH];
	float div_by_0_output_asm[LENGTH];
	float div_by_0_output_c[LENGTH];

	int overflow_result_asm;
	int overflow_result_c;
	int div_by_0_result_asm;
	int div_by_0_result_c;

	/* ----------------------------------------------------------------------
	** Overflow Tester
	** ------------------------------------------------------------------- */
	kalman_state *overflow_test_state = create_kalman_state(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);

	overflow_result_asm = Kalmanfilter_asm(measurements, overflow_output_asm, overflow_test_state, LENGTH);
	overflow_result_c = Kalmanfilter_C(measurements, overflow_output_c, overflow_test_state, LENGTH);

	/* ----------------------------------------------------------------------
	** Divided by zero Tester
	** ------------------------------------------------------------------- */
	kalman_state *div_by_0_test_state = create_kalman_state(0, 0, 0, 0, 0);

	div_by_0_result_asm = Kalmanfilter_asm(measurements, div_by_0_output_asm, div_by_0_test_state, LENGTH);
	div_by_0_result_c = Kalmanfilter_C(measurements, div_by_0_output_c, div_by_0_test_state, LENGTH);


	/* Dealing with Failures */
	if(status == ARM_MATH_TEST_FAILURE){
		while(1);
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1){
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

