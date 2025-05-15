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
#include "DrvGY_MPU60X0.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define READ_MPU60X0_GYRO_ACCEL 0
#define READ_BMP180 0
#define READ_MPU60X0_BOLLA_TEST 0

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
	  HAL_UART_Transmit(&huart2, "Can't start BMP180 driver!", strlen("Can't start BMP180 driver!"), 1000);

  if (DrvMPU60X0Cfg_Init() != UTLGEN_OK)
  	  HAL_UART_Transmit(&huart2, "Can't start MPU60X0 driver!", strlen("Can't start MPU60X0 driver!"), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const uint8_t* line_0 = " ";
  const uint8_t* new_line = "\r\n";

  const uint8_t* line_1 = "Raw_Pres: ";
  const uint8_t* line_2 = "Pressione: ";
  const uint8_t* line_3 = "[Pa] ";

  const uint8_t* line_4 = "| Altitude: ";
  const uint8_t* line_5 = "[m] ";

  const uint8_t* line_6 = "| Temperatura: ";
  const uint8_t* line_7 = " -> ";
  const uint8_t* line_8 = "[C]";

  const uint8_t* GY_X = "GY_X: ";
  const uint8_t* GY_Y = "GY_Y: ";
  const uint8_t* GY_Z = "GY_Z: ";

  const uint8_t* ACC_X = "ACC_X: ";
  const uint8_t* ACC_Y = "ACC_Y: ";
  const uint8_t* ACC_Z = "ACC_Z: ";

  char output_sting_final_temp[20];
  char output_sting_final_pres[20];
  char output_sting_altitude[20];

  char output_sting_final_gx[20];
  char output_sting_final_gy[20];
  char output_sting_final_gz[20];

  char output_sting_final_ax[20];
  char output_sting_final_ay[20];
  char output_sting_final_az[20];

  GY_Data_t GYOutputData;
  ACC_Data_t ACCELOutputData;

  while (1)
  {
  #if READ_BMP180
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

    HAL_UART_Transmit(&huart2, line_2, strlen(line_2), 1000);
	  HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_pres, strlen(output_sting_final_pres), 1000);
	  HAL_UART_Transmit(&huart2, line_3, strlen(line_3), 1000);

  #endif
    
  #if READ_MPU60X0_GYRO_ACCEL
    if(getGyro(&GYOutputData) == UTLGEN_OK)
    {       
      itoa(GYOutputData.XG, output_sting_final_gx, 10);
      HAL_UART_Transmit(&huart2, GY_X, strlen(GY_X), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_gx, strlen(output_sting_final_gx), 1000);
      HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);

      itoa(GYOutputData.YG, output_sting_final_gy, 10);
      HAL_UART_Transmit(&huart2, GY_Y, strlen(GY_Y), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_gy, strlen(output_sting_final_gy), 1000);
      HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
      
      itoa(GYOutputData.ZG, output_sting_final_gz, 10);
      HAL_UART_Transmit(&huart2, GY_Z, strlen(GY_Z), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_gz, strlen(output_sting_final_gz), 1000);
      HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
    }
    else
    {
        HAL_UART_Transmit(&huart2, GY_Z, strlen(GY_Z), 1000);
        HAL_UART_Transmit(&huart2, "Can't get GY!!!", strlen("Can't get GY!!!"), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
    }
    
    if (getAccel(&ACCELOutputData) == UTLGEN_OK)
    {
        itoa(ACCELOutputData.XA, output_sting_final_ax, 10);
        HAL_UART_Transmit(&huart2, ACC_X, strlen(ACC_X), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_ax, strlen(output_sting_final_ax), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);

        itoa(ACCELOutputData.YA, output_sting_final_ay, 10);
        HAL_UART_Transmit(&huart2, ACC_Y, strlen(ACC_Y), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_ay, strlen(output_sting_final_ay), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
      
        itoa(ACCELOutputData.ZA, output_sting_final_az, 10);
        HAL_UART_Transmit(&huart2, ACC_Z, strlen(ACC_Z), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_az, strlen(output_sting_final_az), 1000);
        HAL_UART_Transmit(&huart2, new_line, strlen(new_line), 1000);
    }
    else
    {
        HAL_UART_Transmit(&huart2, GY_Z, strlen(GY_Z), 1000);
        HAL_UART_Transmit(&huart2, "Can't get ACCEL!!!", strlen("Can't get ACCEL!!!"), 1000);
        HAL_UART_Transmit(&huart2, new_line, strlen(new_line), 1000);
    }

    #endif

    #if READ_MPU60X0_BOLLA_TEST
    bool test_bolla;
    isDeviceOnPlainSurface(&test_bolla);
    HAL_UART_Transmit(&huart2, "Is the device in bolla? ", strlen("Is the device in bolla? "), 1000);
    if (test_bolla)
        HAL_UART_Transmit(&huart2, "CIANO XE IN BOLLA!", strlen("CIANO XE IN BOLLA!"), 1000);
    else
        HAL_UART_Transmit(&huart2, "NO A XE IN BOLLA..", strlen("NO A XE IN BOLLA.."), 1000);
    HAL_UART_Transmit(&huart2, new_line, strlen(new_line), 1000);
    
    #endif

	  HAL_Delay(100);

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
