/*
 * DrvGY_HMC5883L.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#include "DrvGY_HMC5883L.h"

static DrvGY_HMC5883L_DelayCB_t mDelayCB;
static DrvGY_HMC5883L_WriteRegisterCB_t mWriteRegisterCB;
static DrvGY_HMC5883L_ReadRegisterCB_t mReadRegisterCB;
static DrvGY_HMC5883L_IsDeviceReadyCB_t mIsDeviceReadyCB;
static DrvGY_HMC5883L_MasterTransmitCB_t mMasterTrasmitCB;
static DrvGY_HMC5883L_MasterReceiveCB_t mMasterReceiveCB;

UtlGen_Err_t DrvHMC5883L_Init(const DrvHMC5883L_Config_t *pConfigData)
{
    mDelayCB = pConfigData->pfDelayCB;
    mWriteRegisterCB = pConfigData->pfWriteRegistersI2cCB;
    mReadRegisterCB = pConfigData->pfReadRegistersI2cCB;
    mIsDeviceReadyCB = pConfigData->pfIsDeviceReadyI2cCB;
    mMasterTrasmitCB = pConfigData->pfMasterTransmitI2cCB;
    mReadRegisterCB = pConfigData->pfMasterReveiceCB;

    // Device time to be ready for I2C commands
    mDelayCB(260);

    return UTLGEN_OK;
}
