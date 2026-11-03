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

  #define SET_NULL 0

  // PWR_MGMT_1 SETTINGS
  #define S_PWR_MGMT_1_DEVICE_RESET               128
  #define S_PWR_MGMT_1_SLEEP                      64
  #define S_PWR_MGMT_1_CYCLE                      32
  #define S_PWR_MGMT_1_TEMP_DIS                   8
  #define S_PWR_MGMT_1_CLKSEL_CLOCK_STOP          7
  #define S_PWR_MGMT_1_CLKSEL_PLL_EXT_19KHZ_REF   5
  #define S_PWR_MGMT_1_CLKSEL_PLL_EXT_32KHZ_REF   4
  #define S_PWR_MGMT_1_CLKSEL_PLL_Z_GYRO_REF      3
  #define S_PWR_MGMT_1_CLKSEL_PLL_Y_GYRO_REF      2
  #define S_PWR_MGMT_1_CLKSEL_PLL_X_GYRO_REF      1
  #define S_PWR_MGMT_1_CLKSEL_INT_8MHZ            0

  // CONFIG SETTINGS
  #define S_DLPF_CFG_260HZ_1MS  0
  #define S_DLPF_CFG_185HZ_2MS  1
  #define S_DLPF_CFG_96HZ_3MS   2
  #define S_DLPF_CFG_43HZ_5MS   3 
  #define S_DLPF_CFG_20HZ_8MS   4
  #define S_DLPF_CFG_10HZ_14MS  5 
  #define S_DLPF_CFG_5HZ_19MS   6

  #define S_EXT_SYNC_INPUT_DISABLED 0
  #define S_EXT_SYNC_TEMP_OUT       8       
  #define S_EXT_SYNC_GYRO_XOUT_L    16
  #define S_EXT_SYNC_GYRO_YOUT_L    24
  #define S_EXT_SYNC_GYRO_ZOUT_L    32
  #define S_EXT_SYNC_ACCEL_XOUT_L   40
  #define S_EXT_SYNC_ACCEL_YOUT_L   48
  #define S_EXT_SYNC_ACCEL_ZOUT_L   56

  // GYRO SETTINGS
  #define S_FS_SEL_250    0
  #define S_FS_SEL_500    8
  #define S_FS_SEL_1000   16
  #define S_FS_SEL_2000   24

  // ACCEL SETTINGS
  #define S_AFS_SEL_2G     0
  #define S_AFS_SEL_4G     8
  #define S_AFS_SEL_8G     16
  #define S_AFS_SEL_16G    24

 /******************************************************************************
  * Module Preprocessor Macros
  ******************************************************************************/

 /******************************************************************************
  * Function Prototypes Private
  ******************************************************************************/

 /******************************************************************************
  * Module Variable Definitions
  ******************************************************************************/
 
  static DrvMPU60X0_Config_t mConfigData; // Configuration Istance
  static DrvMPU60X0_Settings_t mSettingsData; // Settings Istance

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

    // Refer to MPU_60X0_REGISTER_MAP documentation to compile those settings 
    mSettingsData.RESET_STARTUP = S_PWR_MGMT_1_DEVICE_RESET + S_PWR_MGMT_1_SLEEP;
    mSettingsData.PWE_MGMT_1_SETTINGS = S_PWR_MGMT_1_TEMP_DIS + S_PWR_MGMT_1_CLKSEL_PLL_X_GYRO_REF;
    mSettingsData.DLPF_FSYNC_SETTINGS = S_DLPF_CFG_20HZ_8MS + S_EXT_SYNC_INPUT_DISABLED;
    mSettingsData.GYRO_SETTINGS = S_FS_SEL_250;
    mSettingsData.FS_SEL = S_FS_SEL_250;
    mSettingsData.ACCEL_SETTINGS = S_AFS_SEL_2G;
    mSettingsData.AFS_SEL = S_AFS_SEL_2G;
    mSettingsData.SMPRT_DIVIDER = 0; // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 

    ErrCode = DrvGY_MPU60X0_Init(&mConfigData, &mSettingsData);
    return ErrCode;
  }
