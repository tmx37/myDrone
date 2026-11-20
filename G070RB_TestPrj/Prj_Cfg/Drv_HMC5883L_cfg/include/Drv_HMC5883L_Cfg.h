/*
 * Drv_HMC5883L_Cfg.h
 *
 *  Created on: 20/11/2026
 *      Author: tmx37
 */

#ifndef DRV_HMC5883L_CFG_DRV_HMC5883L_CFG_H_
#define DRV_HMC5883L_CFG_DRV_HMC5883L_CFG_H_

#include "GenUtl.h"
#include <stdint.h>

/// @brief Default configuration values
#define HMC_DEFAULT_GAIN             HMC_GAIN_1090
#define HMC_DEFAULT_MEASUREMENT_MODE HMC_SING_MEAS
#define HMC_DEFAULT_AVERAGING        HMC_AVG_8_SAMPLE
#define HMC_DEFAULT_DATARATE         HMC_DR_15_HZ
#define HMC_DEFAULT_BIAS             HMC_BIAS_NORMAL


GenUtl_Err_t Drv_HMC5883LCfg_Init();

#endif /* DRV_HMC5883L_CFG_DRV_HMC5883L_CFG_H_ */
