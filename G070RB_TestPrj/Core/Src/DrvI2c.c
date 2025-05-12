#include "DrvI2c.h"
#include <string.h>
#include "main.h"

// TODO: move this to .h
//extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c1;

uint32_t DrvI2c_ReadRegister(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
{
	return HAL_I2C_Mem_Read(&hi2c1, DevAddr, RegNdx, 1, pDataBuffer, DataSize, HAL_MAX_DELAY); // OK
}

uint32_t DrvI2c_WriteRegister(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
{
	return HAL_I2C_Mem_Write(&hi2c1, DevAddr, RegNdx, 1, pDataBuffer, DataSize, HAL_MAX_DELAY); // OK
}

uint32_t DrvI2c_IsDeviceReady(uint32_t I2cInst, uint8_t DevAddr, uint8_t nTrials)
{
	return HAL_I2C_IsDeviceReady(&hi2c1, DevAddr, nTrials, HAL_MAX_DELAY);
}

// Transmits data to target device in blocking mode
uint32_t DrvI2c_MasterTransmit(uint32_t I2cInst, uint16_t DevAddr, uint8_t *pData, uint16_t DataSize)
{
	return HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, DataSize, HAL_MAX_DELAY);
}

// Receives data from the target device in blocking mode
uint32_t DrvI2c_MasterReceive(uint32_t I2cInst, uint16_t DevAddr, uint8_t *pData, uint16_t DataSize)
{
	return HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, DataSize, HAL_MAX_DELAY);
}

void DrvI2c_Delay(uint32_t Delay)
{
	return HAL_Delay(Delay);
}
