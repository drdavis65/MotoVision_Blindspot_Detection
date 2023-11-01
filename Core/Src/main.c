/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TIMEOUT_MS 1000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t LIDAR_ADDR1 = 0x62;
uint8_t LIDAR_ADDR2 = 0x55;
uint8_t ACQ_COMMAND = 0x00;
uint8_t TAKE_DIST = 0x04;
uint8_t DISTANCE_REG_LOW = 0x10;
uint8_t DISTANCE_REG_HIGH = 0x11;
uint8_t STATUS_REG = 0x01;
uint8_t BOARD_TEMPERATURE = 0xE0;
uint8_t FACTORY_RESET = 0xE4;
uint8_t DETECTION_SENSIVITY = 0x1C;
uint8_t ACQUISITION_COUNT = 0x05;
uint8_t QUICK_TERMINATION = 0xE5;
uint8_t SOC_TEMPERATURE = 0xEC;
uint8_t HIGH_ACCURACY_MODE = 0xEB;
uint8_t status;
uint8_t distance_low;
uint8_t distance_high;
uint16_t distanceL;
uint16_t distanceR;

HAL_StatusTypeDef CheckDevice(uint8_t lidarAddr){
	char msg[128];

	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, lidarAddr << 1, 10, HAL_MAX_DELAY);
	if(ret == HAL_OK)
	{
	  sprintf(msg, "Device is ready.\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
	}
	else
	{
	  sprintf(msg, "Device is not connected.\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
	}
	return ret;
}



void configureLidarAddress(uint8_t lidarliteAddress, uint8_t newAddress, uint8_t disableDefault)
{
	uint8_t dataBytes[5];
    // Enable flash storage
    dataBytes[0] = 0x11;
    HAL_I2C_Mem_Write(&hi2c1, lidarliteAddress << 1, 0xEA, 1, dataBytes, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    // Read 4-byte device serial number
    HAL_I2C_Mem_Read(&hi2c1, lidarliteAddress << 1, 0x16, 1, dataBytes, 4, HAL_MAX_DELAY);

    // Append the desired I2C address to the end of the serial number byte array
    dataBytes[4] = newAddress;

    // Write the serial number and new address in one 5-byte transaction
    HAL_I2C_Mem_Write(&hi2c1, lidarliteAddress << 1, 0x16, 1, dataBytes, 5, HAL_MAX_DELAY);

    // Wait for the I2C peripheral to be restarted with new device address
    HAL_Delay(100);

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefault)
    {
        dataBytes[0] = 0x01; // set bit to disable default address
        HAL_I2C_Mem_Write(&hi2c1, newAddress << 1, 0x1b, 1, dataBytes, 1, HAL_MAX_DELAY);

        // Wait for the I2C peripheral to be restarted with new device address
        HAL_Delay(100);
    }

    // Disable flash storage
    dataBytes[0] = 0;
    HAL_I2C_Mem_Write(&hi2c1, newAddress << 1, 0xEA, 1, dataBytes, 1, HAL_MAX_DELAY);
    HAL_Delay(100);
    CheckDevice(newAddress);
}

// Other main functions and setup go here...


uint16_t GetDistance(uint8_t lidarAddr){
	uint32_t startTick = HAL_GetTick(); // Get current tick for timeout

	// 1. Write 0x04 to register 0x00.
	HAL_I2C_Mem_Write(&hi2c1, lidarAddr << 1, ACQ_COMMAND, 1, &TAKE_DIST, 1, HAL_MAX_DELAY);


	do { // 2. Read register 0x01.
	  HAL_I2C_Mem_Read(&hi2c1, lidarAddr << 1, STATUS_REG, 1, &status, 1, HAL_MAX_DELAY);

	  if((HAL_GetTick() - startTick) > TIMEOUT_MS) {// Handle timeout
	  	    return 0;  // return invalid distance value
	  }

	} while (status & 0x01); // 3. Repeat step 2 until bit 0 (LSB) goes low.


	// 4. Read two bytes from 0x10 (low byte 0x10 then high byte 0x11) to obtain the 16-bit measured distance in centimeters.
	HAL_I2C_Mem_Read(&hi2c1, lidarAddr << 1, DISTANCE_REG_LOW, 1, &distance_low, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, lidarAddr << 1, DISTANCE_REG_HIGH, 1, &distance_high, 1, HAL_MAX_DELAY);

	return (((uint16_t)distance_high << 8) | distance_low);
}

void L_RED_LED() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void L_GREEN_LED() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void L_YELLOW_LED() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void L_OFF_LED() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void R_RED_LED() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
void R_GREEN_LED() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
void R_YELLOW_LED() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
void R_OFF_LED() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//red pin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); 	//green pin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	//blue pin
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  char msg[128];
  //configureLidarAddress(LIDAR_ADDR1, LIDAR_ADDR2, 1);
  while(CheckDevice(LIDAR_ADDR1) != HAL_OK) {
	  sprintf(msg, "device 1\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
  }
  while(CheckDevice(LIDAR_ADDR2) != HAL_OK) {
	  sprintf(msg, "device 2\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  distanceR = GetDistance(LIDAR_ADDR1);
	  distanceL = GetDistance(LIDAR_ADDR2);

	  sprintf(msg, "distance L: %d\r\n", distanceL);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

	  sprintf(msg, "distance R: %d\r\n", distanceR);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);

	  HAL_Delay(100);


	  if(distanceL < 50 && distanceR < 50 && distanceL != 0 && distanceR != 0) {
		  L_GREEN_LED();
		  R_GREEN_LED();
	  }
	  else {
		  if(distanceL < 50)
			  L_RED_LED();
		  else
			  L_YELLOW_LED();
		  if(distanceR < 50)
			  R_RED_LED();
		  else
			  R_YELLOW_LED();
	  }




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 196;
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L_RED_Pin|L_GREEN_Pin|L_BLUE_Pin|R_BLUE_Pin
                          |R_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R_RED_GPIO_Port, R_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L_RED_Pin L_GREEN_Pin L_BLUE_Pin R_BLUE_Pin
                           R_GREEN_Pin */
  GPIO_InitStruct.Pin = L_RED_Pin|L_GREEN_Pin|L_BLUE_Pin|R_BLUE_Pin
                          |R_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : R_RED_Pin */
  GPIO_InitStruct.Pin = R_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R_RED_GPIO_Port, &GPIO_InitStruct);

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
