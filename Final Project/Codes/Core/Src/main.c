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
#define ARM_MATH_CN4
#include "arm_math.h"

#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_qspi.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_SIGMA_TO_DEGREESx10 14286
#define ROAD_WIDTH 4
#define ROAD_LENGTH 10
#define pi 3.14159265
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int MAX_COUNT = 3;

char gyro_buf[100];
char UI_buf[400];

static int ax = 0;
static int ay = 0;
static int az = 0;

float previous_gyro[3];
float gyro_values[3];
int deg_values[3];

int obstacle[4];
int random_value;
int satellite_pos;

int turning_flag = 0;
int obs_pos = -1;
int is_pressed = 0;
int is_printed = 0;

HAL_StatusTypeDef  UART_status;

// UI Definitions
// Demo
/*
1th     |     |     ^     |     |       where obstacle was created
2nd     |     ^     |     ^     |
3rd     ^     |     |     |     ^
4th     |     |     |     |     |
5th     |     |     |     |     |
6th     |     |     |     |     |
7th     |     |     |     |     |
8th     |     |     |     |     |
9th     |     |     |     |     |
10h     |     |     |     |     |       satellite
 */

// each line has 33 characters
// notice "\\" is one character for '\'
char map[400] = "v     |     |     ^     |     |\r\n"
				" v    |     ^     |     ^     |\r\n"
				"  v   ^     |     |     |     ^\r\n"
				"   v  |     |     |     |     |\r\n"
				"    v |     |     |     |     |\r\n"
				"    v |     |     |     |     |\r\n"
				"   v  |     |     |     |     |\r\n"
				"  v   |     |     |     |     |\r\n"
				" v    |     |     |     |     |\r\n"
				"v     |     |     |     |     |\r\n";

char satellite_UI[5] = "=[x]=";

int satellite_pos = 0;

char *welcome = "|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|\r\n"
				"|                             |\r\n"
				"|                             |\r\n"
				"|          Press              |\r\n"
				"|          Blue Button        |\r\n"
				"|          To Start           |\r\n"
				"|                             |\r\n"
				"|                             |\r\n"
				"|                             |\r\n"
				"|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|\r\n";

char *end_1= "|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|\r\n"
			 "|                             |\r\n"
		     "|         Game over!          |\r\n"
		     "|         Your Score:         |\r\n";
char *end_2= "|                             |\r\n"
			 "|         Press               |\r\n"
			 "|         Blue Button         |\r\n"
			 "|         To Restart          |\r\n"
			 "|~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|\r\n";

int score = 0;

char obs_UI[5] = "oOoOo";

// update map when it reach 4
int mapUpdateCounter = 0;
int order = 0;

// 0 welcome message
// 1 start game
// 2 game over
int gameStatus = 0;

// The arrays to store the tones
uint8_t F3[253];
uint8_t G3[225];
uint8_t A3[200];
uint8_t AA3[189];
uint8_t B3[179];

uint8_t C4[169];
uint8_t CC4[159];
uint8_t D4[150];
uint8_t DD4[142];
uint8_t E4[134];
uint8_t F4[126];
uint8_t FF4[120];
uint8_t G4[113];
uint8_t GG4[106];
uint8_t A4[100];
uint8_t AA4[95];
uint8_t B4[89];

uint8_t C5[84];
uint8_t CC5[80];
uint8_t D5[75];
uint8_t DD5[71];
uint8_t E5[67];
uint8_t F5[63];
uint8_t FF5[60];
uint8_t G5[56];
uint8_t GG5[53];
uint8_t A5[50];
uint8_t AA5[47];
uint8_t B5[45];

uint8_t C6[42];
uint8_t CC6[40];
uint8_t D6[38];
uint8_t DD6[35];
uint8_t E6[33];
uint8_t F6[32];
uint8_t FF6[30];
uint8_t G6[28];
uint8_t GG6[27];
uint8_t A6[25];
uint8_t AA6[24];
uint8_t B6[22];

uint8_t C7[21];
uint8_t CC7[20];
uint8_t D7[19];
uint8_t DD7[18];
uint8_t E7[17];
uint8_t F7[16];
uint8_t FF7[15];
uint8_t G7[14];
uint8_t GG7[13];
uint8_t A7[13];
uint8_t AA7[12];
uint8_t B7[11];

uint8_t C8[11];
uint8_t CC8[10];
uint8_t D8[9];
uint8_t DD8[9];
uint8_t E8[8];
uint8_t F8[8];
uint8_t FF8[7];
uint8_t G8[7];
uint8_t GG8[7];
uint8_t A8[6];
uint8_t AA8[6];
uint8_t B8[6];

uint8_t Pause[1];

uint32_t addr = 0x000000;
uint32_t game_music_addr = 0x00000000;
uint32_t slow_music_addr = 0x00020000;
uint32_t music2_addr = 0x00000000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_OCTOSPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Do the  integration for the gyroscope
  *
  * @param sample_x: value of x
  * @param sample_y: value of y
  * @param sample_z: value of z
  *
  * @retval None
  */
void gyroIntegrate(int sample_x, int sample_y, int sample_z)
{
    ax += sample_x - previous_gyro[0];
    ay += sample_y - previous_gyro[1];
    az += sample_z - previous_gyro[2];
}


/**
  * @brief Convert the gyroscope outputs to degree
  *
  *
  * @param xdeg: degree of x = ax * s / r
  * @param ydeg: degree of y = ay * s / r
  * @param zdeg: degree of z = az * s / r
  *
  * @note r is the gyro resolution in degrees per second (0.07)
  * @note s is the sample rate (100)
  * @note GYRO_SIGMA_TO_DEGREESx10 = 100 / 0.07 * 100 = 1428.57 * 10 = 14286
  * @note Times by 100 to change to integer instead of float
  *
  * @retval None
  */
void getAngleXYZ(int *deg_values)
{
	deg_values[0] = (ax * 10) / GYRO_SIGMA_TO_DEGREESx10;
	deg_values[1] = (ay * 10) / GYRO_SIGMA_TO_DEGREESx10;
	deg_values[2] = (az * 10) / GYRO_SIGMA_TO_DEGREESx10;
}


/**
  * @brief Update the map by copying each row
  */
void updateMap()
{
	// Copy the last row and paste to first row later
	char temp[40] = "";
	for (int j = 0; j < 33; j++){
		temp[j] =  map[33 * 9 + j];
	}

	// Copy the first row to the second row, second to third, and third to forth
	int i = 0;
	for (i = 9; i > 0; i--){
		for (int j = 0; j < 33; j++){
			map[33 * i + j] =  map[33 * (i - 1) + j];
		}
	}

	// Update first row
	map[0]  = temp[0];
	map[6]  = temp[6];
	map[12] = temp[12];
	map[18] = temp[18];
	map[24] = temp[24];
	map[30] = temp[30];

	for (int j = 1; j < 6; j++){
		map[j] = temp[j];
		map[6 + j]  = ' ';
		map[12 + j] = ' ';
		map[18 + j] = ' ';
		map[24 + j] = ' ';
	}
}


/**
  * @brief Update the satellite in UI
  *
  * @param x: the new position of the satellite in UI
  *
  */
void updateSatellite(int x){
	// clear the old position first
	for (int i = 0; i < 5; i++){
		map[33 * 9 + 6 + 6 * satellite_pos + 1 + i] = ' ';
	}

	if (x >= 0 && x <= 3){
		for (int j = 0; j < 5; j++){
			map[33 * 9 + 6 + 1 + (6 * x) + j] = satellite_UI[j];
		}
		satellite_pos = x;

	} else {
		updateSatellite(satellite_pos);
	}
}


/**
  * @brief Put a obstacle in UI
  *
  * @param x: the position of the obstacle in UI
  *
  */
void generateObstacle(int track){
	if(track >= 0 && track <= 3){
		for (int j = 0; j < 5; j++){
			map[6 + 6 * track + 1 + j] = obs_UI[j];
		}
	}
}


/**
  * @brief Clean the screen and print the message inputed
  *
  * @param msg: The message to be printed
  *
  */
void printMessage(char *msg){
	// Clear the content first
	memset(UI_buf, 0, strlen(UI_buf));
	sprintf(UI_buf, "\033[10A");
	HAL_UART_Transmit(&huart1, (uint8_t *) UI_buf, (uint16_t) strlen(UI_buf), 30000);

	// Clean UI_buf
	memset(UI_buf, 0, strlen(UI_buf));
	sprintf(UI_buf, msg);
	HAL_UART_Transmit(&huart1, (uint8_t *) UI_buf, strlen(UI_buf), 10000);
}


/**
  * @brief Print the ending UI
  */
void printEndingMessage(){
	memset(UI_buf, 0, strlen(UI_buf));
	sprintf(UI_buf, "\033[2J\033[10A");
	HAL_UART_Transmit(&huart1, (uint8_t *) UI_buf, (uint16_t) strlen(UI_buf), 30000);


	memset(UI_buf, 0, strlen(UI_buf));	//clears UI_buf
	sprintf(UI_buf, end_1);
	HAL_UART_Transmit(&huart1, (uint8_t *) UI_buf, sizeof(UI_buf), 10000);

	memset(UI_buf, 0, strlen(UI_buf));	//clears UI_buf
	sprintf(UI_buf, "|             %4d            |\r\n", score);
	HAL_UART_Transmit(&huart1, (uint8_t *) UI_buf, sizeof(UI_buf), 10000);

	memset(UI_buf, 0, strlen(UI_buf));	//clears UI_buf
	sprintf(UI_buf, end_2);
	HAL_UART_Transmit(&huart1, (uint8_t *) UI_buf, sizeof(UI_buf), 10000);

}


/**
 * @brief Create the tones needed for the music and sound effect
 */
void createTones(){

	// Pause
	for(int i = 0; i < 1; i++){
		Pause[i] = (uint8_t) 0;
	}

	/**
	 * ===================== Octave 3 =====================
	 */

	// Tone F3 = 174.61 Hz
	for(int i = 0; i < 253; i++){
		F3[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 253)) * 256);
	}

	// Tone G3 = 196.00 Hz
	for(int i = 0; i < 225; i++){
		G3[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 225)) * 256);
	}

	// Tone A3 = 220.00 Hz
	for(int i = 0; i < 200; i++){
		A3[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 200)) * 256);
	}

	// Tone AA3 = 233.08 Hz
	for(int i = 0; i < 189; i++){
		AA3[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 189)) * 256);
	}

	// Tone B3 = 246.94 Hz
	for(int i = 0; i < 179; i++){
		B3[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 179)) * 256);
	}

	/**
	 * ===================== Octave 4 =====================
	 */
	// Tone C4 = 261.63 Hz
	for(int i = 0; i < 169; i++){
		C4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 169)) * 256);
	}

	// Tone CC4 = 277.18 Hz
	for(int i = 0; i < 159; i++){
		CC4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 159)) * 256);
	}

	// Tone D4 = 293.66 Hz
	for(int i = 0; i < 150; i++){
		D4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 150)) * 256);
	}

	// Tone DD4 = 311.13 Hz
	for(int i = 0; i < 142; i++){
		DD4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 142)) * 256);
	}

	// Tone E4 = 329.63 Hz
	for(int i = 0; i < 134; i++){
		E4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 134)) * 256);
	}

	// Tone F4 = 349.23 Hz
	for(int i = 0; i < 126; i++){
		F4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 126)) * 256);
	}

	// Tone FF4 = 369.99 Hz
	for(int i = 0; i < 120; i++){
		FF4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 120)) * 256);
	}

	// Tone G4 = 392.00 Hz
	for(int i = 0; i < 113; i++){
		G4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 113)) * 256);
	}

	// Tone GG4 = 415.30 Hz
	for(int i = 0; i < 106; i++){
		GG4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 106)) * 256);
	}

	// Tone A4 = 440.00 Hz
	for(int i = 0; i < 100; i++){
		A4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 100)) * 256);
	}

	// Tone AA4 = 466.16 Hz
	for(int i = 0; i < 95; i++){
		AA4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 95)) * 256);
	}

	// Tone B4 = 493.88 Hz
	for(int i = 0; i < 89; i++){
		B4[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 89)) * 256);
	}

	/**
	 * ===================== Octave 5 =====================
	 */
	// Tone C5 = 523.25 Hz
	for(int i = 0; i < 84; i++){
		C5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 84)) * 256);
	}

	// Tone CC5 = 554.37 Hz
	for(int i = 0; i < 80; i++){
		CC5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 80)) * 256);
	}

	// Tone D5 = 587.33 Hz
	for(int i = 0; i < 75; i++){
		D5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 75)) * 256);
	}

	// Tone DD5 = 622.25 Hz
	for(int i = 0; i < 71; i++){
		DD5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 71)) * 256);
	}

	// Tone E5 = 659.25 Hz
	for(int i = 0; i < 67; i++){
		E5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 67)) * 256);
	}

	// Tone F5 = 698.46 Hz
	for(int i = 0; i < 63; i++){
		F5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 63)) * 256);
	}

	// Tone FF5 = 739.99 Hz
	for(int i = 0; i < 60; i++){
		FF5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 60)) * 256);
	}

	// Tone G5 = 783.99 Hz
	for(int i = 0; i < 56; i++){
		G5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 56)) * 256);
	}

	// Tone GG5 = 830.61 Hz
	for(int i = 0; i < 53; i++){
		GG5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 53)) * 256);
	}

	// Tone A5 = 880.00 Hz
	for(int i = 0; i < 50; i++){
		A5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 50)) * 256);
	}

	// Tone AA5 = 932.33 Hz
	for(int i = 0; i < 47; i++){
		AA5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 47)) * 256);
	}

	// Tone B5 = 987.77 Hz
	for(int i = 0; i < 45; i++){
		B5[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 45)) * 256);
	}

	/**
	 * ===================== Octave 6 =====================
	 */
	// Tone C6 = 1046.50 Hz
	for(int i = 0; i < 42; i++){
		C6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 42)) * 256);
	}

	// Tone CC6 = 1108.73 Hz
	for(int i = 0; i < 40; i++){
		CC6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 40)) * 256);
	}

	// Tone D6 = 1174.66 Hz
	for(int i = 0; i < 38; i++){
		D6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 38)) * 256);
	}

	// Tone DD6 = 1244.51 Hz
	for(int i = 0; i < 35; i++){
		DD6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 35)) * 256);
	}

	// Tone E6 = 1318.51 Hz
	for(int i = 0; i < 33; i++){
		E6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 33)) * 256);
	}

	// Tone F6 = 1396.91 Hz
	for(int i = 0; i < 32; i++){
		F6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 32)) * 256);
	}

	// Tone FF6 = 1479.98 Hz
	for(int i = 0; i < 30; i++){
		FF6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 30)) * 256);
	}

	// Tone G6 = 1567.98 Hz
	for(int i = 0; i < 28; i++){
		G6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 28)) * 256);
	}

	// Tone GG6 = 1661.22 Hz
	for(int i = 0; i < 27; i++){
		GG6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 27)) * 256);
	}

	// Tone A6 = 1760.00 Hz
	for(int i = 0; i < 25; i++){
		A6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 25)) * 256);
	}

	// Tone AA6 = 1864.66 Hz
	for(int i = 0; i < 24; i++){
		AA6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 24)) * 256);
	}

	// Tone B6 = 1975.53 Hz
	for(int i = 0; i < 22; i++){
		B6[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 22)) * 256);
	}

	/**
	 * ===================== Octave 7 =====================
	 */
	// Tone C7 = 2093.00 Hz
	for(int i = 0; i < 21; i++){
		C7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 21)) * 256);
	}

	// Tone CC7 = 2217.46 Hz
	for(int i = 0; i < 20; i++){
		CC7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 20)) * 256);
	}

	// Tone D7 = 2349.32 Hz
	for(int i = 0; i < 19; i++){
		D7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 19)) * 256);
	}

	// Tone DD7 = 2489.02 Hz
	for(int i = 0; i < 18; i++){
		DD7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 18)) * 256);
	}

	// Tone E7 = 2637.02 Hz
	for(int i = 0; i < 17; i++){
		E7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 17)) * 256);
	}

	// Tone F7 = 2793.83 Hz
	for(int i = 0; i < 16; i++){
		F7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 16)) * 256);
	}

	// Tone FF7 = 2959.96 Hz
	for(int i = 0; i < 15; i++){
		FF7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 15)) * 256);
	}

	// Tone G7 = 3135.96 Hz
	for(int i = 0; i < 14; i++){
		G7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 14)) * 256);
	}

	// Tone GG7 = 3322.44 Hz
	for(int i = 0; i < 13; i++){
		GG7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 13)) * 256);
	}

	// Tone A7 = 3520.00 Hz
	for(int i = 0; i < 13; i++){
		A7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 13)) * 256);
	}

	// Tone AA7 = 3729.31 Hz
	for(int i = 0; i < 12; i++){
		AA7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 12)) * 256);
	}

	// Tone B7 = 3951.07 Hz
	for(int i = 0; i < 11; i++){
		B7[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 11)) * 256);
	}

	/**
	 * ===================== Octave 8 =====================
	 */
	// Tone C8 = 4186.01 Hz
	for(int i = 0; i < 11; i++){
		C8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 11)) * 256);
	}

	// Tone CC8 = 4434.92 Hz
	for(int i = 0; i < 10; i++){
		CC8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 10)) * 256);
	}

	// Tone D8 = 4698.63 Hz
	for(int i = 0; i < 9; i++){
		D8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 9)) * 256);
	}

	// Tone DD8 = 4978.03 Hz
	for(int i = 0; i < 9; i++){
		DD8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 9)) * 256);
	}

	// Tone E8 = 5274.04 Hz
	for(int i = 0; i < 8; i++){
		E8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 8)) * 256);
	}

	// Tone F8 = 5587.65 Hz
	for(int i = 0; i < 8; i++){
		F8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 8)) * 256);
	}

	// Tone FF8 = 5919.91 Hz
	for(int i = 0; i < 7; i++){
		FF8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 7)) * 256);
	}

	// Tone G8 = 6271.93 Hz
	for(int i = 0; i < 7; i++){
		G8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 7)) * 256);
	}

	// Tone GG8 = 6644.88 Hz
	for(int i = 0; i < 7; i++){
		GG8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 7)) * 256);
	}

	// Tone A8 = 7040.00 Hz
	for(int i = 0; i < 6; i++){
		A8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 6)) * 256);
	}

	// Tone AA8 = 7458.62 Hz
	for(int i = 0; i < 6; i++){
		AA8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 6)) * 256);
	}

	// Tone B8 = 7902.13 Hz
	for(int i = 0; i < 6; i++){
		B8[i] = (uint8_t) (0.33 * (1 + arm_sin_f32(2 * pi * i / 6)) * 256);
	}
}

void writePause(){
	  for(int i = 0; i < 8; i++){
	  	  if(BSP_QSPI_Write((uint8_t *)Pause, game_music_addr, 1) != QSPI_OK){
	  		  Error_Handler();
	  	  }
	  	game_music_addr += 1;
	  }
}

void writeMegalovaniaQuick(){
	 int factor = 50;

	 for(int i = 0; i < factor; i++){
	  	  if(BSP_QSPI_Write((uint8_t *)D5, game_music_addr, 75) != QSPI_OK){
	  		  Error_Handler();
	  	  }
	  	  game_music_addr += 75;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  if(BSP_QSPI_Write((uint8_t *)D5, game_music_addr, 75) != QSPI_OK){
	  		  Error_Handler();
	  	  }
	  	  game_music_addr += 75;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D6, game_music_addr, 38) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 38;
	  }

	  writePause();

	  for(int i = 0; i < factor * 3; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)A5, game_music_addr, 50) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 50;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)GG5, game_music_addr, 53) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 53;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 56;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, game_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 63;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D5, game_music_addr, 75) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 75;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, game_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 63;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 56;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)C5, game_music_addr, 84) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 84;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)C5, game_music_addr, 84) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 84;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D6, game_music_addr, 38) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 38;
	  }

	  writePause();

	  for(int i = 0; i < factor * 3; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)A5, game_music_addr, 50) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 50;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)GG5, game_music_addr, 53) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 53;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 56;
	  }

	  writePause();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, game_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 63;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D5, game_music_addr, 75) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 75;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, game_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 63;
	  }

	  writePause();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  game_music_addr += 56;
	  }

	  writePause();
}

void writePauseSlow(){
	  for(int i = 0; i < 8; i++){
	  	  if(BSP_QSPI_Write((uint8_t *)Pause, slow_music_addr, 1) != QSPI_OK){
	  		  Error_Handler();
	  	  }
	  	slow_music_addr += 1;
	  }
}

void writeMegalovaniaSlow(){
	 int factor = 100;

	 for(int i = 0; i < factor; i++){
	  	  if(BSP_QSPI_Write((uint8_t *)D5, slow_music_addr, 75) != QSPI_OK){
	  		  Error_Handler();
	  	  }
	  	  slow_music_addr += 75;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  if(BSP_QSPI_Write((uint8_t *)D5, slow_music_addr, 75) != QSPI_OK){
	  		  Error_Handler();
	  	  }
	  	  slow_music_addr += 75;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D6, slow_music_addr, 38) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 38;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 3; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)A5, slow_music_addr, 50) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 50;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)GG5, slow_music_addr, 53) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 53;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, slow_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 56;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, slow_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 63;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D5, slow_music_addr, 75) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 75;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, slow_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 63;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, slow_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 56;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)C5, slow_music_addr, 84) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 84;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)C5, slow_music_addr, 84) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 84;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D6, slow_music_addr, 38) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 38;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 3; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)A5, slow_music_addr, 50) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 50;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)GG5, slow_music_addr, 53) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 53;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, slow_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 56;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor * 2; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, slow_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 63;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)D5, slow_music_addr, 75) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 75;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)F5, slow_music_addr, 63) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 63;
	  }

	  writePauseSlow();

	  for(int i = 0; i < factor; i++){
	  	  	  if(BSP_QSPI_Write((uint8_t *)G5, slow_music_addr, 56) != QSPI_OK){
	  	  		  Error_Handler();
	  	  	  }
		  slow_music_addr += 56;
	  }

	  writePauseSlow();
}

void writeStartMenuMusic(){
	int factor = 100;

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)G4, game_music_addr, 113) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 113;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 56;
	}

	writePause();

	for(int i = 0; i < factor * 2; i++){
		if(BSP_QSPI_Write((uint8_t *)D5, game_music_addr, 75) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 75;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)C5, game_music_addr, 84) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 84;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 56;
	}

	writePause();

	for(int i = 0; i < factor * 2; i++){
		if(BSP_QSPI_Write((uint8_t *)G4, game_music_addr, 113) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 113;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)G4, game_music_addr, 113) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 113;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)C5, game_music_addr, 84) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 84;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 56;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)A5, game_music_addr, 50) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 50;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)G5, game_music_addr, 56) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 56;
	}

	writePause();

	for(int i = 0; i < factor; i++){
		if(BSP_QSPI_Write((uint8_t *)D5, game_music_addr, 75) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 75;
	}

	writePause();

	for(int i = 0; i < factor * 2; i++){
		if(BSP_QSPI_Write((uint8_t *)C5, game_music_addr, 84) != QSPI_OK){
			Error_Handler();
		}
		game_music_addr += 84;
	}

	writePause();

}

/**
 * @brief Reset the game values
 */
void gameReset(){
	ax = 0;
	ay = 0;
	az = 0;
	satellite_pos = 0;
	turning_flag = 0;
	score = 0;
	obs_pos = -1;

	for (int i = 0; i <= 2; i++){
		previous_gyro[i] = 0;
		gyro_values[i] = 0;
		deg_values[i] = 0;
	}

	for (int j = 0; j <= 3; j++){
		obstacle[j] = 0;
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
  // Initialize the sensors
  BSP_GYRO_Init();
  BSP_QSPI_Init();

  gyro_values[0] = 0;
  gyro_values[1] = 0;
  gyro_values[2] = 0;

  createTones();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  for(int i = 0; i < 10; i++){
  	  if(BSP_QSPI_Erase_Block(addr + i * 0x00010000) != QSPI_OK){
  		  Error_Handler();
  	  }
  }

  uint8_t megalovania_slow[184760] = {0};
  uint8_t megalovania_quick[92460] = {0};

  writeMegalovaniaSlow();
  writeMegalovaniaQuick();

  // Start the DAC and Timer
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  //184760
  //98688
  //92460

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/**
		 * =============================== Update UI ===============================
		 */

		// In the start state
		if (gameStatus == 0){

			// If the welcome page haven't been printed, print it.
			if (is_printed == 0){
				is_printed = 1;
				gameReset();
				printMessage(welcome);
			}

			// If the button is pressed
			if (is_pressed == 1){
				is_pressed = 0;
				is_printed = 0;
				gameStatus = 1;
				MAX_COUNT = 3;

				if(BSP_QSPI_Read((uint8_t *)megalovania_slow, 0x00020000, 184760) != QSPI_OK){
					Error_Handler();
				}

				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, megalovania_slow, 184760, DAC_ALIGN_8B_R);
			}

		// In the playing state
		} else if (gameStatus == 1) {
			if (mapUpdateCounter == MAX_COUNT - 1) {
				if (MAX_COUNT == 3 && score >= 50){
					MAX_COUNT--;
				}

				if (MAX_COUNT == 2 && score >= 150){
					MAX_COUNT--;

					if(BSP_QSPI_Read((uint8_t *)megalovania_quick, 0x00000000, 92460) != QSPI_OK){
						Error_Handler();
					}

					HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
					HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, megalovania_quick, 92460, DAC_ALIGN_8B_R);
				}

				updateMap();
				score++;

				// If a new obstacle is needed, create a new obstacle
				if (obs_pos < 0) {
					// Create an integer between 0 and ROAD_WIDTH.
					random_value = rand() % ROAD_WIDTH;
					obstacle[random_value] = ROAD_LENGTH;
					obs_pos = random_value;
					generateObstacle(obs_pos);

				}else {
					// else, update the obstacle position
					obstacle[obs_pos] -= 1;

					if(obstacle[obs_pos] <= 0){
						obs_pos = -1;
					}
				}
			}

			// Check if the game is over, and send the flag
			if(obstacle[satellite_pos] == 1) {
				gameStatus = 2;
			}

			mapUpdateCounter = (mapUpdateCounter + 1) % MAX_COUNT;

			/**
			 * =============================== Do Gyroscope Read ===============================
			 */
			// Store previous values
			previous_gyro[0] = gyro_values[0];
			previous_gyro[1] = gyro_values[1];
			previous_gyro[2] = gyro_values[2];

			// Read the sensors
			BSP_GYRO_GetXYZ(gyro_values);

			// Convert the values into degree changed
			gyroIntegrate(gyro_values[0], gyro_values[1], gyro_values[2]);
			getAngleXYZ(deg_values);

			// Clean the buffer for storing the outputs
			memset(gyro_buf, '\0', strlen(gyro_buf));

			// Do the gyroscope turning detection
			if(deg_values[2] <= -100){
//				sprintf(gyro_buf, "TURN RIGHT!\r\n");
				turning_flag = 1;

			}else if(deg_values[2] >= 100){
//				sprintf(gyro_buf, "TURN LEFT!\r\n");
				turning_flag = -1;

			}else{
//				sprintf(gyro_buf, "Gyro (x, y, z): (%d, %d, %d)\r\nDegree Changed (x, y, z): (%d, %d, %d)\r\n",
//								(int) gyro_values[0], (int) gyro_values[1], (int) gyro_values[2],
//								(int) deg_values[0], (int) deg_values[1], (int) deg_values[2]);
				turning_flag = 0;
			}

			/**
			 * =============================== Satellite Update ===============================
			 */
			int pos_to_update = satellite_pos;

			if (turning_flag == 1) pos_to_update = (satellite_pos + 1) > 3 ? 3 : satellite_pos + 1;
			else if (turning_flag == -1) pos_to_update = (satellite_pos - 1) < 0 ? 0 : satellite_pos - 1;

			updateSatellite(pos_to_update);
			printMessage(map);

			// When turned, give a signal by LED 1 and wait for 180ms
			if(turning_flag != 0){
				HAL_GPIO_TogglePin(My_LED_GPIO_Port, My_LED_Pin);
				turning_flag = 0;
//				HAL_Delay(100);
				HAL_GPIO_TogglePin(My_LED_GPIO_Port, My_LED_Pin);
			}

			if(is_pressed == 1){
				is_pressed = 0;
			}

		// In the game over state
		} else if (gameStatus == 2) {
			if(is_printed == 0){
				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
				is_printed = 1;
				printEndingMessage();
			}

			if(is_pressed == 1){
				is_pressed = 0;
				is_printed = 0;
				gameStatus = 0;
				score = 0;
			}
		}

		// Wait for 10ms to achieve 100Hz
		HAL_Delay(10);
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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
  htim2.Init.Period = 1814;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(My_LED_GPIO_Port, My_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : My_Button_Pin */
  GPIO_InitStruct.Pin = My_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(My_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : My_LED_Pin */
  GPIO_InitStruct.Pin = My_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(My_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// GPIO Callback Function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ((int) GPIO_Pin == (int) My_Button_Pin) {
		is_pressed = 1;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
