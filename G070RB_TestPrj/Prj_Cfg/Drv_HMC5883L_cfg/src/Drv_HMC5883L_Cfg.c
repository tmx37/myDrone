/*
 * DrvGY_HMC5883L_Cfg.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#include "Drv_HMC5883L_Cfg.h"
#include "Drv_HMC5883L.h"
#include "main.h"

static DrvHMC5883L_Config_t mConfigData;

GenUtl_Err_t Drv_HMC5883LCfg_Init()
{
    GenUtl_Err_t ErrCode;
    mConfigData.pfWriteRegistersI2cCB = HAL_I2C_Mem_Write;
    mConfigData.pfReadRegistersI2cCB = HAL_I2C_Mem_Read;
    mConfigData.pfIsDeviceReadyI2cCB = HAL_I2C_IsDeviceReady;
    mConfigData.pfMasterTransmitI2cCB = HAL_I2C_Master_Transmit;
    mConfigData.pfMasterReveiceCB = HAL_I2C_Master_Receive;
    mConfigData.pfDelayCB = HAL_Delay;

    ErrCode = Drv_HMC5883L_Init(&mConfigData);
    return ErrCode;
}
