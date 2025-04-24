#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t DrvI2c_ReadRegister(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);

uint32_t DrvI2c_WriteRegister(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);

uint32_t DrvI2c_IsDeviceReady(uint32_t I2cInst, uint8_t DevAddr, uint8_t nTrials);


#ifdef __cplusplus
}
#endif
