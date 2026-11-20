/*
 * DrvGY_HMC5883L_Cfg.c
 *
 *  Created on: 20/11/2026
 *      Author: tmx37
 */

#include "Drv_HMC5883L_Cfg.h"
#include "Drv_HMC5883L.h"
#include "main.h"


static Drv_HMC5883L_Cfg_t mConfigData;

GenUtl_Err_t I2C_Mem_Write_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
GenUtl_Err_t I2C_Mem_Read_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
GenUtl_Err_t I2C_IsDeviceReady_Wrapper(uint8_t DevAddr, uint8_t nTrials);
GenUtl_Err_t I2C_MasterTransmit_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
GenUtl_Err_t I2C_MasterReceive_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

GenUtl_Err_t Drv_HMC5883LCfg_Init()
{
    GenUtl_Err_t ErrCode;
    mConfigData.pfWriteRegistersI2cCB = I2C_Mem_Write_Wrapper;
    mConfigData.pfReadRegistersI2cCB = I2C_Mem_Read_Wrapper;
    mConfigData.pfIsDeviceReadyI2cCB = I2C_IsDeviceReady_Wrapper;
    mConfigData.pfMasterTransmitI2cCB = I2C_MasterTransmit_Wrapper;
    mConfigData.pfMasterReveiceCB = I2C_MasterReceive_Wrapper;
    mConfigData.pfDelayCB = HAL_Delay;

    ErrCode = Drv_HMC5883L_Init(&mConfigData);
    return ErrCode;
}

GenUtl_Err_t I2C_Mem_Write_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
{
    /** TODO: forward to LM_Hw_Comms */
    (void)DevAddr;
    (void)RegNdx;
    (void)pDataBuffer;
    (void)DataSize;
    return OUT_OK;
}

GenUtl_Err_t I2C_Mem_Read_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
{
    /** TODO: forward to LM_Hw_Comms */
    (void)DevAddr;
    (void)RegNdx;
    (void)pDataBuffer;
    (void)DataSize;
    return OUT_OK;
}

GenUtl_Err_t I2C_IsDeviceReady_Wrapper(uint8_t DevAddr, uint8_t nTrials)
{
    /** TODO: forward to LM_Hw_Comms */
    (void)DevAddr;
    (void)nTrials;
    return OUT_OK;
}

GenUtl_Err_t I2C_MasterTransmit_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    /** TODO: forward to LM_Hw_Comms */
    (void)DevAddress;
    (void)pData; 
    (void)Size;
    return OUT_OK;
}

GenUtl_Err_t I2C_MasterReceive_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    /** TODO: forward to LM_Hw_Comms */
    (void)DevAddress;
    (void)pData; 
    (void)Size;
    return OUT_OK;
}

