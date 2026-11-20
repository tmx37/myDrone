/*
 * Drv_MPU6000_Cfg.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

  /******************************************************************************
 * Includes
 *******************************************************************************/

 #include "Drv_MPU60X0_Cfg.h"
 #include "Drv_MPU60X0.h"

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
 
  static Drv_MPU60X0_Config_t mConfigData; // Configuration Istance
  static Drv_MPU60X0_Settings_t mSettingsData; // Settings Istance

 /******************************************************************************
  * Function Definitions Private
  ******************************************************************************/

  GenUtl_Err_t I2C_Mem_Write_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
  GenUtl_Err_t I2C_Mem_Read_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);
  GenUtl_Err_t I2C_IsDeviceReady_Wrapper(uint8_t DevAddr, uint8_t nTrials);
  GenUtl_Err_t I2C_MasterTrasmit_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
  GenUtl_Err_t I2C_MasterReceive_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

 /******************************************************************************
  * Function Definitions Public
  ******************************************************************************/

  GenUtl_Err_t  Drv_MPU60X0Cfg_Init(void)
  {
    GenUtl_Err_t ErrCode;
    mConfigData.pfWriteRegistersI2cCB = I2C_Mem_Write_Wrapper;
    mConfigData.pfReadRegisterI2cCB = I2C_Mem_Read_Wrapper;
    mConfigData.pfIsDeviceReadyI2cCB = I2C_IsDeviceReady_Wrapper;
    mConfigData.pfMasterTransmitCB = I2C_MasterTrasmit_Wrapper;
    mConfigData.pfMasterReceiveCB = I2C_MasterReceive_Wrapper;
    mConfigData.pfDelayCB = HAL_Delay;

    // Refer to MPU_60X0_REGISTER_MAP documentation to compile those settings 
    mSettingsData.RESET_STARTUP = DRV_MPU60X0_S_PWR_MGMT_1_DEVICE_RESET + DRV_MPU60X0_S_PWR_MGMT_1_SLEEP;
    mSettingsData.PWE_MGMT_1_SETTINGS = DRV_MPU60X0_S_PWR_MGMT_1_TEMP_DIS + DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_PLL_X_GYRO_REF;
    mSettingsData.DLPF_FSYNC_SETTINGS = DRV_MPU60X0_S_DLPF_CFG_20HZ_8MS + DRV_MPU60X0_S_EXT_SYNC_INPUT_DISABLED;
    mSettingsData.GYRO_SETTINGS = DRV_MPU60X0_S_FS_SEL_250;
    mSettingsData.FS_SEL = DRV_MPU60X0_S_FS_SEL_250;
    mSettingsData.ACCEL_SETTINGS = DRV_MPU60X0_S_AFS_SEL_2G;
    mSettingsData.AFS_SEL = DRV_MPU60X0_S_AFS_SEL_2G;
    mSettingsData.SMPRT_DIVIDER = 0; // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 

    ErrCode = Drv_MPU60X0_Init(&mConfigData, &mSettingsData);
    return ErrCode;
  }

  GenUtl_Err_t I2C_Mem_Write_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
  {
    /** TODO: forward to LM_Comms */
    (void)DevAddr;
    (void)RegNdx; 
    (void)pDataBuffer; 
    (void)DataSize;
    return OUT_OK;
  }

  GenUtl_Err_t I2C_Mem_Read_Wrapper(uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize)
  {
    /** TODO: forward to LM_Comms */
    (void)DevAddr;
    (void)RegNdx; 
    (void)pDataBuffer; 
    (void)DataSize;
    return OUT_OK;
  }
  
  GenUtl_Err_t I2C_IsDeviceReady_Wrapper(uint8_t DevAddr, uint8_t nTrials)
  {
    /** TODO: forward to LM_Comms */
    (void)DevAddr;
    (void)nTrials;
    return OUT_OK;
  }
  
  GenUtl_Err_t I2C_MasterTrasmit_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
  {
    /** TODO: forward to LM_Comms */
    (void)DevAddress;
    (void)pData;
    (void)Size;
    return OUT_OK;
  }
  
  GenUtl_Err_t I2C_MasterReceive_Wrapper(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
  {
    /** TODO: forward to LM_Comms */
    (void)DevAddress; 
    (void)pData;
    (void)Size;
    return OUT_OK;
  }
  