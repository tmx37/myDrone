/*
 * DrvGY_MPU6000.h
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#ifndef DRVGY_MPU6000_INC_DRVGY_MPU6000_H_
#define DRVGY_MPU6000_INC_DRVGY_MPU6000_H_

#include "DrvGY_BMP180_Cfg.h"

// TODO: find another solution to this
static const uint8_t MPU60X0_ADDR = 0x0068;

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*DrvGY_MPU60X0_WriteRegisterCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
typedef uint32_t (*DrvGY_MPU60X0_ReadRegisterCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
typedef uint32_t (*DrvGY_MPU60X0_IsDeviceReadyCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t nTrials);
typedef uint32_t (*DrvGY_MPU60X0_MasterTransmitCB_t)(uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
typedef uint32_t (*DrvGY_MPU60X0_MasterReceiveCB_t)(uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
typedef void (*DrvGY_MPU60X0_DelayCB_t)(uint32_t Delay);

typedef struct 
{
    uint8_t I2CInst;
    DrvGY_MPU60X0_WriteRegisterCB_t pfWriteRegistersI2cCB;
	DrvGY_MPU60X0_ReadRegisterCB_t pfReadRegisterI2cCB;
	DrvGY_MPU60X0_IsDeviceReadyCB_t pfIsDeviceReadyI2cCB;
    DrvGY_MPU60X0_MasterTransmitCB_t pfMasterTransmitCB;
    DrvGY_MPU60X0_MasterReceiveCB_t pfMasterReceiveCB;
    DrvGY_MPU60X0_DelayCB_t pfDelayCB;


} DrvMPU60X0_Config_t;

typedef struct 
{
    int16_t XG, YG, ZG;
} GY_Data;

typedef struct 
{
    int16_t XA, YA, ZA;
} ACC_Data;

typedef struct 
{
    int16_t XR, YR, ZR;
} XYZ_Angles;

/*
 > INITIALIZE ONLY ONCE SURE YOU ARE IN A PLAIN SURFACE (analyze IIS2ICLX 2-Axis Digital Inclinometer) 
 > Try an accellerometer based approach: if only Z ha accellerometer value = 1, its fine. Find out if it is possible to have a greater resolution.
 > Use FreeRTOS because this library introduces Delays and use that will slow down a "linear" execution system
 */
UtlGen_Err_t DrvGY_MPU60X0_Init(const DrvMPU60X0_Config_t *pConfigData);

UtlGen_Err_t setSamplingTime(uint8_t mseconds);

UtlGen_Err_t getGyro(GY_Data *output);

UtlGen_Err_t getAngles(XYZ_Angles *output);

UtlGen_Err_t getAccel(ACC_Data *output);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DRVGY_MPU6000_INC_DRVGY_MPU6000_H_ */
