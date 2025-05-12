/*
 * DrvGY_MPU6000.h
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#ifndef DRVGY_MPU6000_INC_DRVGY_MPU6000_H_
#define DRVGY_MPU6000_INC_DRVGY_MPU6000_H_

#include "DrvGY_BMP180_Cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*DrvGY_MPU60X0_WriteRegisterCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
typedef uint32_t (*DrvGY_MPU60X0_ReadRegisterCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
typedef uint32_t (*DrvGY_MPU60X0_IsDeviceReadyCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t nTrials);
typedef uint32_t (*DrvGY_MPU60X0_MasterTransmitCB_t)(uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
typedef uint32_t (*DrvGY_MPU60X0_MasterReceiveCB_t)(uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);

typedef struct 
{
    uint8_t I2CInst;
    DrvGY_MPU60X0_WriteRegisterCB_t pfWriteRegistersI2cCB;
	DrvGY_MPU60X0_ReadRegisterCB_t pfReadRegisterI2cCB;
	DrvGY_MPU60X0_IsDeviceReadyCB_t pfIsDeviceReadyI2cCB;
    DrvGY_MPU60X0_MasterTransmitCB_t pfMasterTransmitCB;
    DrvGY_MPU60X0_MasterReceiveCB_t pfMasterReceiveCB;

} DrvMPU60X0_Config_t;


UtlGen_Err_t DrvGY_MPU60X0_Init(const DrvMPU60X0_Config_t *pConfigData);

// TODO: UtlGen_Err setSamplingTime(uint8_t mseconds);
UtlGen_Err_t setSamplingTime(uint8_t mseconds);

// TODO: UtlGen_Err getGyro(&GYData output); 
UtlGen_Err_t getGyro();

// TODO: UtlGen_Err getAngles(&XYZAngles output); 
UtlGen_Err_t getAngles();

// TODO: UtlGen_Err getAcc(&ACCData output); 
UtlGen_Err_t getAccel();

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DRVGY_MPU6000_INC_DRVGY_MPU6000_H_ */
