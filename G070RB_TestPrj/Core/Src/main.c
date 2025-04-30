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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DrvGY_BMP180.h"
#include "DrvGY_MPU6000.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MPU6000_SLAVE_0 0x0068

#define MPU6000_CONFIG 0x1A

#define MPU6000_SELFTEST_X_ADDR 0x0D
#define MPU6000_SELFTEST_Y_ADDR 0x0E
#define MPU6000_SELFTEST_Z_ADDR 0x0F

#define MPU6000_SELFTEST_A_ADDR 0x10

#define MPU6000_ACC_X_OUT_H 0x3B
#define MPU6000_ACC_X_OUT_L 0x3C
#define MPU6000_ACC_Y_OUT_H 0x3D
#define MPU6000_ACC_Y_OUT_L 0x3E
#define MPU6000_ACC_Z_OUT_H 0x3F
#define MPU6000_ACC_Z_OUT_L 0x40

#define MPU6000_GYRO_X_OUT_H 0x43
#define MPU6000_GYRO_X_OUT_L 0x44
#define MPU6000_GYRO_Y_OUT_H 0x45
#define MPU6000_GYRO_Y_OUT_L 0x46
#define MPU6000_GYRO_Z_OUT_H 0x47
#define MPU6000_GYRO_Z_OUT_L 0x48

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  int32_t CH1_DC = 0;

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  if(DrvGYBMP180Cfg_Init() != UTLGEN_OK)
	  HAL_UART_Transmit(&huart2, "AAAAAAAAAAAAAAAAAAA", strlen("AAAAAAAAAAAAAAAAAAA"), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t* line_0 = " ";

  uint8_t* line_1 = "Raw_Pres: ";
  uint8_t* line_2 = "Pressione: ";
  uint8_t* line_3 = "[Pa] ";

  uint8_t* line_4 = "| Altitude: ";
  uint8_t* line_5 = "[m] ";

  uint8_t* line_6 = "| Temperatura: ";
  uint8_t* line_7 = " -> ";
  uint8_t* line_8 = "[C]";
  uint8_t* new_line = "\r\n";

  char output_sting_final_temp[20];
  char output_sting_final_pres[20];
  char output_sting_altitude[20];

  uint8_t test_WhoAmI[8] = {0};

  uint8_t gyro_x_raw[2] = {0};
  uint8_t gyro_y_raw[2] = {0};
  uint8_t gyro_z_raw[2] = {0};

  uint16_t gyro_x_final_value = 0;

  char gyro_x_value_out [20];

  uint8_t messageDataToTransfer[2];

  messageDataToTransfer[0] = 0x6B;
	messageDataToTransfer[1] = 0xC0;			//Register restart value

	HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageDataToTransfer[0], 2, 10);
	HAL_Delay(110);

	messageDataToTransfer[0] = 0x6B;
	messageDataToTransfer[1] = 0x00;			//Register wake up value
	HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageDataToTransfer[0], 2, 10);


  uint8_t messageConfig[2];
  messageConfig[0] = MPU6000_CONFIG;
  messageConfig[1] = 4; // rate di sampling, definito su datasheet
  
  HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageConfig[0], 2, 10);

  // Configure the GYRO_CONFIG register for sensitivity (e.g., ±250 degrees/second)
  messageConfig[0] = 0x1B; // GYRO_CONFIG register
  messageConfig[1] = 0x00; // Set sensitivity to ±250 degrees/second
  HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageConfig[0], 2, 10);

  while (1)
  {
    // while (CH1_DC < 65535)
    // {
    //   TIM1->CCR1 = CH1_DC;
    //   CH1_DC += 70;
    //   HAL_Delay(1);
    // }

    // while (CH1_DC > 0)
    // {
    //   TIM1->CCR1 = CH1_DC;
    //   CH1_DC -= 70;
    //   HAL_Delay(1);
    // }
    
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6000_SLAVE_0, 5, HAL_MAX_DELAY))
    {
      // Correct the register address transmission for gyroscope X-axis high byte
      uint8_t gyro_x_out_h = MPU6000_GYRO_X_OUT_H;
      HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_x_out_h, 1, 50);
      HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_x_raw[0], 2, 10);

      // Combine high and low bytes and apply scaling
      gyro_x_final_value = (gyro_x_raw[0] << 8) + gyro_x_raw[1];
      if (gyro_x_final_value >= 32767) gyro_x_final_value = gyro_x_final_value - 65536;

      // Scale the raw value based on sensitivity (±250 degrees/second = 131 LSB/°/s)
      float gyro_x_scaled = gyro_x_final_value / 131.0;

      // Convert scaled value to string and transmit
      // snprintf(gyro_x_value_out, sizeof(gyro_x_value_out), "%.2f", gyro_x_scaled);
      itoa(gyro_x_final_value, gyro_x_value_out, 10);
      HAL_UART_Transmit(&huart2, (uint8_t*)gyro_x_value_out, strlen(gyro_x_value_out), 1000);
    }

    itoa(getTemperature(), output_sting_final_temp, 10);
	  itoa(getPressure(), output_sting_final_pres, 10);
	  itoa(getAltitude(), output_sting_altitude, 10);

	  HAL_UART_Transmit(&huart2, line_2, strlen(line_2), 1000);
	  HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_pres, strlen(output_sting_final_pres), 1000);
	  HAL_UART_Transmit(&huart2, line_3, strlen(line_3), 1000);

	  HAL_UART_Transmit(&huart2, line_4, strlen(line_4), 1000);
	  HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_altitude, strlen(output_sting_altitude), 1000);
	  HAL_UART_Transmit(&huart2, line_5, strlen(line_5), 1000);

	  HAL_UART_Transmit(&huart2, line_6, strlen(line_6), 1000);
	  HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_temp, strlen(output_sting_final_temp), 1000);
	  HAL_UART_Transmit(&huart2, line_8, strlen(line_8), 1000);

	  HAL_UART_Transmit(&huart2, (uint8_t*)new_line, strlen(new_line), 1000);

	  HAL_Delay(500);

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
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00E0A6F2;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
