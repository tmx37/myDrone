#include "DrvI2c.h"
#include <string.h>
#include "main.h"


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
