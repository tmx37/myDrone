/*
 * DrvGY_HMC5883L_Cfg.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#include "DrvGY_HMC5883L_Cfg.h"
#include "DrvGY_HMC5883L.h"

static DrvHMC5883L_Config_t mConfigData;

UtlGen_Err_t DrvHMC5883LCfg_Init()
{
    UtlGen_Err_t ErrCode;
    mConfigData.I2CInst = 0;
    mConfigData.pfWriteRegistersI2cCB = DrvI2c_WriteRegister;
    mConfigData.pfReadRegistersI2cCB = DrvI2c_ReadRegister;
    mConfigData.pfIsDeviceReadyI2cCB = DrvI2c_IsDeviceReady;
    mConfigData.pfMasterTransmitI2cCB = DrvI2c_MasterTransmit;
    mConfigData.pfMasterReveiceCB = DrvI2c_MasterReceive;
    mConfigData.pfDelayCB = DrvI2c_Delay;

    ErrCode = DrvHMC5883L_Init(&mConfigData);
    return UTLGEN_OK;
}