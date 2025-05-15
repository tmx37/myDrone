/*
 * DrvGY_HMC5883L.h
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#ifndef DRVGY_HMC5883L_INC_DRVGY_HMC5883L_H_
#define DRVGY_HMC5883L_INC_DRVGY_HMC5883L_H_

#include "DrvGY_HMC5883L_Cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*DrvGY_HMC5883L_WriteRegisterCB_t) (uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
typedef uint32_t (*DrvGY_HMC5883L_ReadRegisterCB_t) (uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
typedef uint32_t (*DrvGY_HMC5883L_MasterTransmitCB_t) (uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
typedef uint32_t (*DrvGY_HMC5883L_MasterReceiveCB_t) (uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
typedef uint32_t (*DrvGY_HMC5883L_IsDeviceReadyCB_t) (uint32_t I2cInst, uint8_t DevAddr, uint8_t nTrials);
typedef uint32_t (*DrvGY_HMC5883L_DelayCB_t) (uint32_t Delay);

typedef struct
{
    uint8_t I2CInst;
    DrvGY_HMC5883L_WriteRegisterCB_t pfWriteRegistersI2cCB;
    DrvGY_HMC5883L_ReadRegisterCB_t pfReadRegistersI2cCB;
    DrvGY_HMC5883L_IsDeviceReadyCB_t pfIsDeviceReadyI2cCB;
    DrvGY_HMC5883L_MasterTransmitCB_t pfMasterTransmitI2cCB;
    DrvGY_HMC5883L_MasterReceiveCB_t pfMasterReveiceCB;
    DrvGY_HMC5883L_DelayCB_t pfDelayCB;
} DrvHMC5883L_Config_t;

UtlGen_Err_t DrvHMC5883L_Init(const DrvHMC5883L_Config_t *pConfigData);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DRVGY_HMC5883L_INC_DRVGY_HMC5883L_H_ */
