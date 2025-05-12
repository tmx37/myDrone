#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t DrvI2c_ReadRegister(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);

uint32_t DrvI2c_WriteRegister(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);

uint32_t DrvI2c_IsDeviceReady(uint32_t I2cInst, uint8_t DevAddr, uint8_t nTrials);

// Transmits data to target device in blocking mode
uint32_t DrvI2c_MasterTransmit(uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);

// Receives data from the target device in blocking mode
uint32_t DrvI2c_MasterReceive(uint32_t I2cInst, uint16_t DevAddress, uint8_t *pData, uint16_t Size);


#ifdef __cplusplus
}
#endif
