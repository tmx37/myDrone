/**
 * @file HMC5883L.h
 * @brief Driver interface for HMC5883L 3-Axis Digital Compass Module.
 *
 * Provides enums, configuration structures, and function prototypes for
 * initializing, configuring, and reading data from the HMC5883L magnetometer.
 *
 * @author iek
 * @date March 28, 2025
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

/**
 * @brief  HAL library include for STM32 platform.
 *
 * @note   The user must include the correct HAL header file based on their MCU family.
 *         For example:
 *         @code
 *         #include "stm32g0xx_hal.h"   // For STM32G0 series
 *         #include "stm32f4xx_hal.h"   // For STM32F4 series
 *         #include "stm32l4xx_hal.h"   // For STM32L4 series
 *         @endcode
 *         Make sure to match this include with your STM32 device.
 */
#include "stm32g0xx_hal.h"

/**
 * @brief  I2C handle used for communication with the HMC5883L sensor.
 *
 * @note   The user must define the I2C handle according to the hardware setup.
 *         For example, if I2C1 is used:
 *         @code
 *         extern I2C_HandleTypeDef hi2c1;
 *         @endcode
 *         Replace 'hi2c1' with the appropriate I2C handle name used in your project.
 */
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief  Internal pointer to the I2C handler used by the HMC5883L driver.
 *
 * @note   By default, this points to &hi2c1. If a different I2C instance is used
 *         (e.g., hi2c2 or hi2c3), update this pointer accordingly before using the driver.
 *
 *         Example:
 *         @code
 *         static I2C_HandleTypeDef *hmc_port = &hi2c2;
 *         @endcode
 */
static I2C_HandleTypeDef *hmc_port = &hi2c1;

/// @brief Default configuration values
#define HMC_DEFAULT_GAIN             HMC_GAIN_1090
#define HMC_DEFAULT_MEASUREMENT_MODE HMC_SING_MEAS
#define HMC_DEFAULT_AVERAGING        HMC_AVG_8_SAMPLE
#define HMC_DEFAULT_DATARATE         HMC_DR_15_HZ
#define HMC_DEFAULT_BIAS             HMC_BIAS_NORMAL

/**
 * @enum hmc_adr_t
 * @brief I2C address options for HMC5883L
 */
typedef enum {
	HMC_7_BIT_ADR = 0x1E, HMC_WRITE_ADR = 0x3C, HMC_READ_ADR = 0x3D
} hmc_adr_t;

/**
 * @enum hmc_reg_adr_t
 * @brief Register map for HMC5883L
 */
typedef enum {
	HMC_CON_REG_A,
	HMC_CON_REG_B,
	HMC_MODE_REG,
	HMC_X_MSB,
	HMC_X_LSB,
	HMC_Z_MSB,
	HMC_Z_LSB,
	HMC_Y_MSB,
	HMC_Y_LSB,
	HMC_STATUS_REG,
	HMC_IDE_REG_A,
	HMC_IDE_REG_B,
	HMC_IDE_REG_C
} hmc_reg_adr_t;

/**
 * @enum hmc_measurement_mode_t
 * @brief Operating modes of HMC5883L
 */
typedef enum {
	HMC_CONTI_MEAS = 0x00,  ///< Continuous measurement mode
	HMC_SING_MEAS = 0x01,  ///< Single measurement mode
	HMC_IDLE_1_MEAS = 0x02,  ///< Idle mode 1
	HMC_IDLE_2_MEAS = 0x03   ///< Idle mode 2
} hmc_measurement_mode_t;

/**
 * @enum hmc_bias_t
 * @brief Bias (measurement mode) configuration
 */
typedef enum {
	HMC_BIAS_NORMAL = 0x00, HMC_BIAS_POSITIVE = 0x01, HMC_BIAS_NEGATIVE = 0x02
} hmc_bias_t;

/**
 * @enum hmc_avg_t
 * @brief Sample averaging options
 */
typedef enum {
	HMC_AVG_1_SAMPLE = 0x00,
	HMC_AVG_2_SAMPLE = 0x01,
	HMC_AVG_4_SAMPLE = 0x02,
	HMC_AVG_8_SAMPLE = 0x03
} hmc_avg_t;

/**
 * @enum hmc_datarate_t
 * @brief Output data rate options (Hz)
 */
typedef enum {
	HMC_DR_0_75_HZ = 0x00,
	HMC_DR_1_5_HZ = 0x01,
	HMC_DR_3_HZ = 0x02,
	HMC_DR_7_5_HZ = 0x03,
	HMC_DR_15_HZ = 0x04,
	HMC_DR_30_HZ = 0x05,
	HMC_DR_75_HZ = 0x06
} hmc_datarate_t;

/**
 * @enum hmc_gain_t
 * @brief Gain configuration settings and their resolution
 */
typedef enum {
	HMC_GAIN_1370 = 0x00, ///< ±0.88 Gauss, 0.73 mG/LSb
	HMC_GAIN_1090 = 0x20, ///< ±1.3  Gauss, 0.92 mG/LSb (default)
	HMC_GAIN_820 = 0x40, ///< ±1.9  Gauss, 1.22 mG/LSb
	HMC_GAIN_660 = 0x60, ///< ±2.5  Gauss, 1.52 mG/LSb
	HMC_GAIN_440 = 0x80, ///< ±4.0  Gauss, 2.27 mG/LSb
	HMC_GAIN_390 = 0xA0, ///< ±4.7  Gauss, 2.56 mG/LSb
	HMC_GAIN_330 = 0xC0, ///< ±5.6  Gauss, 3.03 mG/LSb
	HMC_GAIN_230 = 0xE0  ///< ±8.1  Gauss, 4.35 mG/LSb
} hmc_gain_t;

/**
 * @struct hmc_t
 * @brief Structure holding full HMC5883L state and configuration
 */
typedef struct {
	int16_t compass_x;                    ///< X-axis magnetic field data
	int16_t compass_z;                    ///< Z-axis magnetic field data
	int16_t compass_y;                    ///< Y-axis magnetic field data
	hmc_measurement_mode_t mode;         ///< Current measurement mode
	hmc_gain_t gain;                     ///< Current gain setting
	hmc_avg_t averaging;                 ///< Averaging configuration
	hmc_datarate_t datarate;             ///< Output data rate
	hmc_bias_t bias;                     ///< Bias configuration
} hmc_t;

/**
 * @brief Sends raw data to the specified register over I2C.
 */
void HMC_SEND_DATA(hmc_adr_t address, hmc_reg_adr_t reg_address, uint8_t *pData,
		uint16_t size);

/**
 * @brief Reads X, Y, Z magnetic data and updates the compass struct.
 */
void HMC_Read_Compass(hmc_t *compass);

/**
 * @brief Reads measurement mode (Mode Register) and updates compass struct.
 */
void HMC_Read_MEASUR_MODE(hmc_t *compass);

/**
 * @brief Sets measurement mode (Mode Register).
 */
void HMC_Set_MEASUR_MODE(hmc_measurement_mode_t mode);

/**
 * @brief Configures averaging, output data rate and bias in Configuration Register A.
 */
void HMC_Set_Reg_A(hmc_avg_t avg, hmc_datarate_t dr, hmc_bias_t bias);

/**
 * @brief Reads Configuration Register A and updates corresponding fields in compass struct.
 */
void HMC_Read_Reg_A(hmc_t *compass);

/**
 * @brief Reads gain setting from Configuration Register B and updates compass struct.
 */
void HMC_Read_Gain(hmc_t *compass);

/**
 * @brief Sets gain in Configuration Register B.
 */
void HMC_Set_Gain(hmc_gain_t gain);

/**
 * @brief Triggers a single measurement in Single Measurement Mode.
 */
void HMC_Make_Measurement_4_Single(void);

/**
 * @brief Initializes the HMC5883L sensor with default settings.
 */
void HMC5883L_Init(void);

/**
 * @brief Reads all configuration and magnetic field data into the given struct.
 */
void HMC_Read_All(hmc_t *compass);

/**
 * @brief Verifies the identity of the HMC5883L device by checking ID registers.
 * @retval HAL_OK if ID matches, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HMC5883L_Check_ID(void);

#endif /* INC_HMC5883L_H_ */
