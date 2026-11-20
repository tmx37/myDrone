 /*
 * DrvGY_BMP180_Cfg.c
 *
 *  Created on: 20/11/2026
 *      Author: tmx37
 */


 /******************************************************************************
 * Includes
 *******************************************************************************/

#include "Drv_BMP180_Cfg.h"
#include "Drv_BMP180.h"

 /******************************************************************************
  * Module Preprocessor Constants
  ******************************************************************************/

 /******************************************************************************
  * Module Preprocessor Macros
  ******************************************************************************/

 /******************************************************************************
  * Function Prototypes Private
  ******************************************************************************/

  GenUtl_Err_t I2C_Mem_Write_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
  GenUtl_Err_t I2C_Mem_Read_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
  GenUtl_Err_t I2C_IsDeviceReady_Wrapper(uint8_t DevAddr, uint8_t nTrials);

 /******************************************************************************
  * Module Variable Definitions
  ******************************************************************************/

static Drv_BMP180_Cfg_t mConfigData;

 /******************************************************************************
  * Function Definitions Private
  ******************************************************************************/

 /******************************************************************************
  * Function Definitions Public
  ******************************************************************************/

GenUtl_Err_t DrvGYBMP180Cfg_Init(void)
{
	GenUtl_Err_t ErrCode;
	mConfigData.pfWriteRegistersI2cCB = I2C_Mem_Write_Wrapper;
	mConfigData.pfReadRegisterI2cCB = I2C_Mem_Read_Wrapper;
	mConfigData.pfIsDeviceReadyI2cCB = I2C_IsDeviceReady_Wrapper;
	mConfigData.oversampling_setting = 3;

	ErrCode = Drv_BMP180_Init(&mConfigData);
	return ErrCode;
}


GenUtl_Err_t I2C_Mem_Write_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
{
	/** TODO: add here call to LM_Comms */
	(void)DevAddr;
	(void)RegNdx;
	(void)pDataBuffer;
	(void)DataSize;
	return OUT_OK;
}

GenUtl_Err_t I2C_Mem_Read_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
{
	/** TODO: add here call to LM_Comms */
	(void)DevAddr;
	(void)RegNdx;
	(void)pDataBuffer;
	(void)DataSize;
	return OUT_OK;
}

GenUtl_Err_t I2C_IsDeviceReady_Wrapper(uint8_t DevAddr, uint8_t nTrials)
{
	/** TODO: add here call to LM_Comms */
	(void)DevAddr;
	(void)nTrials;
	return OUT_OK;
}


/*** End of File **************************************************************/
