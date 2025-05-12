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

#define MPU6000_SLAVE_0 0x0068

#define GYRO_RESET_SIGNAL 4
#define ACCEL_RESET_SIGNAL 2
#define TEMP_RESET_SIGNAL 1

#define MPU6000_SELFTEST_X_ADDR 0x0D
#define MPU6000_SELFTEST_Y_ADDR 0x0E
#define MPU6000_SELFTEST_Z_ADDR 0x0F
#define MPU6000_SELFTEST_A_ADDR 0x10

#define MPU6000_SAMPLE_RATE_DIV 0x19

#define MPU6000_CONFIG 0x1A

#define MPU6000_SMPRT_DIV 0x25

#define MPU6000_GYRO_CONFIG 0x27
#define MPU6000_ACCL_CONFIG 0x28

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

#define MPU6000_SIGNAL_PATH_RESET 0x68

#define MPU6000_PWE_MGMT_1 0x6B
#define MPU6000_PWE_MGMT_2 0x6C

#define MPU6000_WHO_AM_I 0x75

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
  uint8_t* new_line = "\r\n";

  uint8_t* line_1 = "Raw_Pres: ";
  uint8_t* line_2 = "Pressione: ";
  uint8_t* line_3 = "[Pa] ";

  uint8_t* line_4 = "| Altitude: ";
  uint8_t* line_5 = "[m] ";

  uint8_t* line_6 = "| Temperatura: ";
  uint8_t* line_7 = " -> ";
  uint8_t* line_8 = "[C]";

  uint8_t* GY_X = "GY_X: ";
  uint8_t* GY_Y = "GY_Y: ";
  uint8_t* GY_Z = "GY_Z: ";

  uint8_t* ACC_X = "ACC_X: ";
  uint8_t* ACC_Y = "ACC_Y: ";
  uint8_t* ACC_Z = "ACC_Z: ";


  char output_sting_final_temp[20];
  char output_sting_final_pres[20];
  char output_sting_altitude[20];

  uint8_t test_WhoAmI[8] = {0};

  uint8_t gyro_x_raw[2] = {0};
  uint8_t gyro_y_raw[2] = {0};
  uint8_t gyro_z_raw[2] = {0};

  uint8_t acc_x_raw[2] = {0};
  uint8_t acc_y_raw[2] = {0};
  uint8_t acc_z_raw[2] = {0};

  int16_t gyro_x_final_value = 0;
  int16_t gyro_y_final_value = 0;
  int16_t gyro_z_final_value = 0;

  int16_t acc_x_final_value = 0;
  int16_t acc_y_final_value = 0;
  int16_t acc_z_final_value = 0;

  char gyro_x_value_out [20];
  char gyro_y_value_out [20];
  char gyro_z_value_out [20];

  char acc_x_value_out [20];
  char acc_y_value_out [20];
  char acc_z_value_out [20];

  uint8_t messageDataToTransfer[2];

  uint8_t gyro_x_out_h = MPU6000_GYRO_X_OUT_H;
  uint8_t gyro_y_out_h = MPU6000_GYRO_Y_OUT_H;
  uint8_t gyro_z_out_h = MPU6000_GYRO_Z_OUT_H;

  uint8_t acc_x_out_h = MPU6000_ACC_X_OUT_H;
  uint8_t acc_y_out_h = MPU6000_ACC_Y_OUT_H;
  uint8_t acc_z_out_h = MPU6000_ACC_Z_OUT_H;

  int gyro_x_scaled, gyro_y_scaled, gyro_z_scaled;
  int acc_x_scaled, acc_y_scaled, acc_z_scaled;

  messageDataToTransfer[0] = 0x6B;      // Power Management 1 (?) 
	messageDataToTransfer[1] = 0xC0;			// Register restart value -> reset & sleep

	HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageDataToTransfer[0], 2, 10);
	HAL_Delay(110);

	messageDataToTransfer[0] = 0x6B;      // Power Management 1 (?) 
	messageDataToTransfer[1] = 0x00;			// Reset value
	HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageDataToTransfer[0], 2, 10);

  uint8_t messageConfig[2];
  messageConfig[0] = MPU6000_CONFIG;
  messageConfig[1] = 4; // rate di sampling, definito su datasheet
  
  HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageConfig[0], 2, 10);

  // GYRO_CONFIG register for sensitivity (±250 degrees/second)
  messageConfig[0] = 0x1B; // GYRO_CONFIG register
  messageConfig[1] = 0x00; // Set sensitivity to ±250 degrees/second
  HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageConfig[0], 2, 10);

  // ACC_CONFIG register for sensitivity (±2g)
  messageConfig[0] = 0x1C;
  messageConfig[1] = 0x00; // -> puts AFS_SEL = 0 and sets the sensitivity
  HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &messageConfig[0], 2, 10);

  /* TO IMPLEMENT (in MPU60x0 Driver)
    - understand why I have to work with a shifted slaved device address

    - UtlGen_Err getDMPData(&DMPData output);
    - UtlGen_Err getGyro(&GYData output); 
    - UtlGen_Err getGyro(&ACCData output); 
    - UtlGen_Err getAngles(&XYZAngles output); 
    
    - UtlGen_Err setSamplingTime(uint8_t mseconds);
    - 
    
    - UtlGen_Err MPU6000_init();
      > "register" default angle (signal to call this only when its certain you start on a plain surface)
      > 
  */

  //HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &who_am_i_rx[0], 1, 50);

  while (1)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6000_SLAVE_0, 5, HAL_MAX_DELAY))
    {
    
      #if 0
      // ############################################################## WHO_AM_I ##############################################################
      uint8_t who_am_i_rx[1];
      HAL_I2C_Mem_Read(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_WHO_AM_I, 1, who_am_i_rx, 1, HAL_MAX_DELAY);

      // ############################################################## SIGNAL_PATH_RESET ##############################################################
      // gyro reset
      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_SIGNAL_PATH_RESET, 1, GYRO_RESET_SIGNAL, 1, HAL_MAX_DELAY) == HAL_OK) { HAL_UART_Transmit(&huart2, "Gyro reset!", strlen("Gyro reset!"), 1000); } else { HAL_UART_Transmit(&huart2, "Can't reset Gyro!", strlen("Can't reset Gyro!"), 1000); } 
      // accl reset
      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_SIGNAL_PATH_RESET, 1, ACCEL_RESET_SIGNAL, 1, HAL_MAX_DELAY) == HAL_OK) { HAL_UART_Transmit(&huart2, "Accel reset!", strlen("Accel reset!"), 1000); } else { HAL_UART_Transmit(&huart2, "Can't reset Accel!", strlen("Can't reset Accel!"), 1000); }
      // temp reset
      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_SIGNAL_PATH_RESET, 1, TEMP_RESET_SIGNAL, 1, HAL_MAX_DELAY) == HAL_OK) { HAL_UART_Transmit(&huart2, "Temp reset!", strlen("Temp reset!"), 1000); } else { HAL_UART_Transmit(&huart2, "Can't reset Temp!", strlen("Can't reset Temp!"), 1000); }
      HAL_UART_Transmit(&huart2, "\n\r", strlen("\n\r"), 1000);
      
      // ############################################################## PWR_MGMT_1 ##############################################################

      // "CLKSEL": Clock Source select -> REMEMBER: " [...]t is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source) as the clock reference for improved stability [...] " (from documentation)
      uint8_t CLKSEL_Self8MHz = 0;
      uint8_t CLKSEL_PLLXGyroRef = 1;
      uint8_t CLKSEL_PLLYGyroRef = 2;
      uint8_t CLKSEL_PLLZGyroRef = 3;
      uint8_t CLKSEL_PLLEXT_36768Hz_Ref = 4;
      uint8_t CLKSEL_PLLEXT_19200KHz_Ref = 5;
      uint8_t StopClock = 7;
      
      // "TEMP_DIS": Disables the temperature sensor if set to "1"
      uint8_t DisableTemperatureSentor = 8;

      // "CYCLE": When "SLEEP" is disabled and this is set to "1", puts the the MPU-60x0 to sleep mode and wakes it up to di a single read at a rate determined by LP_WAKE_CTRL (defined in PWR_MGMT_2, at register 108)
      uint8_t SetCycleOn = 32;

      // "SLEEP": When set to 1, puts MPU-60x0 to sleep mode
      uint8_t SetSleepOn = 64;

      // "DEVICE_RESET": When set to 1, resets all internal registers to their default values, then this bit automatically clears to 0 once the reset is done.
      uint8_t DeviceReset = 128;

      // to read (try better)
      uint8_t old_pwr_mgmt = 0;

      // to write
      uint8_t pwr_mgmt_reset = 0;
      uint8_t pwr_mngmt_1_setting = CLKSEL_Self8MHz + DisableTemperatureSentor;

      HAL_I2C_Mem_Read(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_PWE_MGMT_1, 1, &old_pwr_mgmt, 1, HAL_MAX_DELAY);

      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_PWE_MGMT_1, 1, &pwr_mngmt_1_setting, 1, HAL_MAX_DELAY) == HAL_OK) 
      { 
        HAL_UART_Transmit(&huart2, "Internal clock reference set", strlen("Internal clock reference set"), HAL_MAX_DELAY);
      } 
      else
      {
        HAL_UART_Transmit(&huart2, "Cant set internal clock reference set", strlen("Cant set internal clock reference set"), HAL_MAX_DELAY);
      }

      // ############################################################## PWR_MGMT_2 ##############################################################

      // can be used with PWR_MGMT_1 to enable confugurations such as Accelerometer Only low power mode (CYCLE=1 + SLEEP=0 + TEMP_DIS=1 + STBY_XG,STBY_YG,STBY_ZG=1)

      // "STBY_ZG": puts the Z axis gyroscope into standby mode 
      uint8_t STBY_ZG = 0;
      // "STBY_YG": puts the Y axis gyroscope into standby mode 
      uint8_t STBY_ZG = 0;
      // "STBY_XG": puts the X axis gyroscope into standby mode
      uint8_t STBY_ZG = 0;
      
      // "STBY_ZA": puts the Z axis accelerometer into standby mode
      uint8_t STBY_ZG = 0;
      // "STBY_YA": puts the Y axis accelerometer into standby mode
      uint8_t STBY_ZG = 0;
      // "STBY_XA": puts the X axis accelerometer into standby mode
      uint8_t STBY_ZG = 0;
      
      // "LP_WAKE_CTRL": Wake-ups frequency (Used by "CYCLE" setting of PWR_MGMT_1, register 107)
      uint8_t LP_WAKE_CTRL_1n25Hz = 0;
      uint8_t LP_WAKE_CTRL_5Hz = 64;
      uint8_t LP_WAKE_CTRL_20Hz = 128;
      uint8_t LP_WAKE_CTRL_40Hz = 192;

      // to read (try better)
      uint8_t old_pwr_mgmt = 0;

      // to write
      uint8_t pwr_mgmt_reset = 0;
      uint8_t pwr_mngmt_1_setting = 0;

      HAL_I2C_Mem_Read(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_PWE_MGMT_1, 1, &old_pwr_mgmt, 1, HAL_MAX_DELAY);

      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_PWE_MGMT_1, 1, &pwr_mngmt_1_setting, 1, HAL_MAX_DELAY) == HAL_OK) 
      { 
        HAL_UART_Transmit(&huart2, "Internal clock reference set", strlen("Internal clock reference set"), HAL_MAX_DELAY);
      } 
      else
      {
        HAL_UART_Transmit(&huart2, "Cant set internal clock reference set", strlen("Cant set internal clock reference set"), HAL_MAX_DELAY);
      }    
        
      // ############################################################## SMPRT_DIV ##############################################################
      // imposta il rate di sampling dei registri valore dei sensori, della coda fifo e di DMP
      // sampling rate = Gyroscope output rate / (1 + SMPLRT_DIV)

      uint8_t SampleRateDivider = 0;

      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_SMPRT_DIV, 1, &SampleRateDivider, 1, HAL_MAX_DELAY) == HAL_OK)
      {
        HAL_UART_Transmit(&huart2, "Sample rate configured correctly!", strlen("Sample rate configured correctly!"), HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, "Sample rate not configured!", strlen("Sample rate not configured!"), HAL_MAX_DELAY);
      }

            // ############################################################## GYROSCOPE_READ ############################################################## 
      // 250gradi/s, 0,5 "sampling rate", measured = +40, inclinazione attuale = ?
      // variazione_gradi = +40g/s * 0,5s = +20gradi
      HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_x_out_h, 1, 50);
      HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_x_raw[0], 2, 10);

      HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_y_out_h, 1, 50);
      HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_y_raw[0], 2, 10);

      HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_z_out_h, 1, 50);
      HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &gyro_z_raw[0], 2, 10);

      // Combine high and low bytes and apply scaling
      gyro_x_final_value = (gyro_x_raw[0] << 8) + gyro_x_raw[1];
      if (gyro_x_final_value >= 32767) gyro_x_final_value = gyro_x_final_value - 65536;
      
      gyro_y_final_value = (gyro_y_raw[0] << 8) + gyro_y_raw[1];
      if (gyro_y_final_value >= 32767) gyro_y_final_value = gyro_y_final_value - 65536;

      gyro_z_final_value = (gyro_z_raw[0] << 8) + gyro_y_raw[1];
      if (gyro_z_final_value >= 32767) gyro_z_final_value = gyro_z_final_value - 65536;

      // Scale the raw value based on sensitivity (±250 degrees/second = 131 LSB/°/s)
      gyro_x_scaled = gyro_x_final_value / 131;
      gyro_y_scaled = gyro_y_final_value / 131;
      gyro_z_scaled = gyro_z_final_value / 131;

      // ############################################################## ACCELLEROMETER READ ############################################################## 
      // 1g = 9,8m/s2, 0,5s "sampling rate", mesured = 2g, m_percorsi = ?
      // m = 2g * s2 -> 2(9.8m/s2) * (0,5s)2 = 19.6m percorso idealmente (?)
      HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &acc_x_out_h, 1, 50);
      HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &acc_x_raw[0], 2, 10);

      HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &acc_y_out_h, 1, 50);
      HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &acc_y_raw[0], 2, 10);

      HAL_I2C_Master_Transmit(&hi2c1, MPU6000_SLAVE_0 << 1, &acc_z_out_h, 1, 50);
      HAL_I2C_Master_Receive(&hi2c1, MPU6000_SLAVE_0 << 1, &acc_z_raw[0], 2, 10);

      // Combine high and low bytes and apply scaling
      acc_x_final_value = (acc_x_raw[0] << 8) + acc_x_raw[1];
      if (acc_x_final_value >= 32767) acc_x_final_value = acc_x_final_value - 65536;
            
      acc_y_final_value = (acc_y_raw[0] << 8) + acc_y_raw[1];
      if (acc_y_final_value >= 32767) acc_y_final_value = acc_y_final_value - 65536;
      
      acc_z_final_value = (acc_z_raw[0] << 8) + acc_y_raw[1];
      if (acc_z_final_value >= 32767) acc_z_final_value = acc_z_final_value - 65536;
      
      // Scale the raw value based on sensitivity (±2g = 16384 LSB/g)
      acc_x_scaled = acc_x_final_value / 16384;
      acc_y_scaled = acc_y_final_value / 16384;
      acc_z_scaled = acc_z_final_value / 16384;

      // PRINT VALUES
      // Convert scaled value to string and transmit
      itoa(gyro_x_scaled, gyro_x_value_out, 10);
  	  HAL_UART_Transmit(&huart2, GY_X, strlen(GY_X), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)gyro_x_value_out, strlen(gyro_x_value_out), 1000);
  	  HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);

      itoa(gyro_y_scaled, gyro_y_value_out, 10);
  	  HAL_UART_Transmit(&huart2, GY_Y, strlen(GY_Y), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)gyro_y_value_out, strlen(gyro_y_value_out), 1000);
      HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
  
      itoa(gyro_z_scaled, gyro_z_value_out, 10);
  	  HAL_UART_Transmit(&huart2, GY_Z, strlen(GY_Z), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)gyro_z_value_out, strlen(gyro_z_value_out), 1000);
      HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);

      // Convert scaled value to string and transmit
      itoa(acc_x_scaled, acc_x_value_out, 10);
      HAL_UART_Transmit(&huart2, ACC_X, strlen(ACC_X), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)acc_x_value_out, strlen(acc_x_value_out), 1000);
      HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);

      itoa(acc_y_scaled, acc_y_value_out, 10);
      HAL_UART_Transmit(&huart2, ACC_Y, strlen(ACC_Y), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)acc_y_value_out, strlen(acc_y_value_out), 1000);
      HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
  
      itoa(acc_z_scaled, acc_z_value_out, 10);
      HAL_UART_Transmit(&huart2, ACC_Z, strlen(ACC_Z), 1000);
      HAL_UART_Transmit(&huart2, (uint8_t*)acc_z_value_out, strlen(acc_z_value_out), 1000);
      HAL_UART_Transmit(&huart2, new_line, strlen(new_line), 1000);

    // ############################################################## GYRO_CONFIG ##############################################################

      // Triggers gyroscope self-test and configures gyroscope full scale range.

      // "FS_SEL": selects the full scale range of the gyroscope output
      uint8_t FS_SEL_250 = 0;
      uint8_t FS_SEL_500 = 8;
      uint8_t FS_SEL_1000 = 16;
      uint8_t FS_SEL_2000 = 24;

      // "ZG_ST": activates and perform self-test on Z axis of the gyroscope
      uint8_t ZG_SelfTest = 32;

      // "YG_ST": activates and perform self-test on Y axis of the gyroscope
      uint8_t YG_SelfTest = 64;

      // "XG_ST": activates and perform self-test on X axis of the gyroscope
      uint8_t XG_SelfTest = 128;

      // Settings value to write
      uint8_t GyroConfig_settings = FS_SEL_250;

      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_GYRO_CONFIG, 1, &GyroConfig_settings, 1, HAL_MAX_DELAY) == HAL_OK)
      {
        HAL_UART_Transmit(&huart2, "Gyroscope configured correctly!", strlen("Gyroscope configured correctly!"), HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, "Gyroscope not configured!", strlen("Gyroscope not configured!"), HAL_MAX_DELAY);
      }

      // ############################################################## ACCEL_CONFIG ##############################################################
    
      // Triggers accellerometer self-test and configures gyroscope full scale range.

      // "FS_SEL": selects the full scale range of the gyroscope output
      uint8_t AFS_SEL_2g = 0;
      uint8_t AFS_SEL_4g = 8;
      uint8_t AFS_SEL_8g = 16;
      uint8_t AFS_SEL_16g = 24;

      // "ZA_ST": activates and perform self-test on Z axis of the accellerometer
      uint8_t ZA_SelfTest = 32;

      // "YA_ST": activates and perform self-test on Y axis of the accellerometer
      uint8_t YA_SelfTest = 64;

      // "XA_ST": activates and perform self-test on X axis of the accellerometer
      uint8_t XA_SelfTest = 128;

      // Settings value to write
      uint8_t AccelConfig_settings = AFS_SEL_2g;

      if (HAL_I2C_Mem_Write(&hi2c1, MPU6000_SLAVE_0 << 1, MPU6000_ACCL_CONFIG, 1, &AccelConfig_settings, 1, HAL_MAX_DELAY) == HAL_OK)
      {
        HAL_UART_Transmit(&huart2, "Accellerometer configured correctly!", strlen("Accellerometer configured correctly!"), HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, "Accellerometer not configured!", strlen("Accellerometer not configured!"), HAL_MAX_DELAY);
      }

      #endif
      
      


    }

    // itoa(getTemperature(), output_sting_final_temp, 10);
	  // itoa(getPressure(), output_sting_final_pres, 10);
	  // itoa(getAltitude(), output_sting_altitude, 10);

	  // HAL_UART_Transmit(&huart2, line_2, strlen(line_2), 1000);
	  // HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_pres, strlen(output_sting_final_pres), 1000);
	  // HAL_UART_Transmit(&huart2, line_3, strlen(line_3), 1000);

	  // HAL_UART_Transmit(&huart2, line_4, strlen(line_4), 1000);
	  // HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_altitude, strlen(output_sting_altitude), 1000);
	  // HAL_UART_Transmit(&huart2, line_5, strlen(line_5), 1000);

	  // HAL_UART_Transmit(&huart2, line_6, strlen(line_6), 1000);
	  // HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_temp, strlen(output_sting_final_temp), 1000);
	  // HAL_UART_Transmit(&huart2, line_8, strlen(line_8), 1000);

	  // HAL_UART_Transmit(&huart2, (uint8_t*)new_line, strlen(new_line), 1000);

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
