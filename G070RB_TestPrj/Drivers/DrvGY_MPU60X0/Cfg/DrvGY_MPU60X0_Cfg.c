/*
 * DrvGY_MPU6000_Cfg.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

  /******************************************************************************
 * Includes
 *******************************************************************************/

 #include "DrvGY_MPU60X0_Cfg.h"
 #include "DrvGY_MPU60X0.h"

 /******************************************************************************
  * Module Preprocessor Constants
  ******************************************************************************/

 /******************************************************************************
  * Module Preprocessor Macros
  ******************************************************************************/

 /******************************************************************************
  * Function Prototypes Private
  ******************************************************************************/

 /******************************************************************************
  * Module Variable Definitions
  ******************************************************************************/
 
 // TODO: add configuration instance  
  static DrvMPU60X0_Config_t mConfigData;

 /******************************************************************************
  * Function Definitions Private
  ******************************************************************************/

 /******************************************************************************
  * Function Definitions Public
  ******************************************************************************/

  UtlGen_Err_t  DrvMPU60X0Cfg_Init(void)
  {
    UtlGen_Err_t ErrCode;
    mConfigData.I2CInst = 0;
    mConfigData.pfWriteRegistersI2cCB = DrvI2c_WriteRegister;
    mConfigData.pfReadRegisterI2cCB = DrvI2c_ReadRegister;
    mConfigData.pfIsDeviceReadyI2cCB = DrvI2c_IsDeviceReady;
    mConfigData.pfMasterTransmitCB = DrvI2c_MasterTransmit;
    mConfigData.pfMasterReceiveCB = DrvI2c_MasterReceive;
    mConfigData.pfDelayCB = DrvI2c_Delay;

    // TODO: Configure other general settings

    ErrCode = DrvGY_MPU60X0_Init(&mConfigData);
    return ErrCode;
  }
