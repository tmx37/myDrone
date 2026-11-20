/*
 * Drv_MPU6000_Cfg.h
 *
 *  Created on: 20/11/2026
 *      Author: tmx37
 */

#ifndef DRV_MPU6000_CFG_DRV_MPU6000_CFG_H_
#define DRV_MPU6000_CFG_DRV_MPU6000_CFG_H_

 /******************************************************************************
 * Includes
 ******************************************************************************/

 #include "GenUtl.h"
 #include "main.h"

 /******************************************************************************
 * Preprocessor Constants
 ******************************************************************************/

    #define DRV_MPU60X0_ADDR 0x0068

    #define DRV_MPU60X0_ACCEL_CONFIG 0x1C
    #define DRV_MPU60X0_ACCEL_X_OUT_H 0x3B
    #define DRV_MPU60X0_ACCEL_X_OUT_L 0x3C
    #define DRV_MPU60X0_ACCEL_Y_OUT_H 0x3D
    #define DRV_MPU60X0_ACCEL_Y_OUT_L 0x3E
    #define DRV_MPU60X0_ACCEL_Z_OUT_H 0x3F
    #define DRV_MPU60X0_ACCEL_Z_OUT_L 0x40

    #define DRV_MPU60X0_GYRO_CONFIG 0x1B
    #define DRV_MPU60X0_GYRO_X_OUT_H 0x43
    #define DRV_MPU60X0_GYRO_X_OUT_L 0x44
    #define DRV_MPU60X0_GYRO_Y_OUT_H 0x45
    #define DRV_MPU60X0_GYRO_Y_OUT_L 0x46
    #define DRV_MPU60X0_GYRO_Z_OUT_H 0x47
    #define DRV_MPU60X0_GYRO_Z_OUT_L 0x48

    #define DRV_MPU60X0_SAMPLE_RATE_DIV 0x19
    #define DRV_MPU60X0_CONFIG 0x1A
    #define DRV_MPU60X0_SMPRT_DIV 0x25
    #define DRV_MPU60X0_SIGNAL_PATH_RESET 0x68
    #define DRV_MPU60X0_GYRO_RESET_SIGNAL 4
    #define DRV_MPU60X0_ACCEL_RESET_SIGNAL 2

    #define DRV_MPU60X0_PWE_MGMT_1 0x6B
    #define DRV_MPU60X0_PWE_MGMT_2 0x6C

    #define DRV_MPU60X0_WHO_AM_I 0x75

 /******************************************************************************
 * Configuration Constants
 ******************************************************************************/

    // PWR_MGMT_1 SETTINGS
    #define DRV_MPU60X0_S_PWR_MGMT_1_DEVICE_RESET               128
    #define DRV_MPU60X0_S_PWR_MGMT_1_SLEEP                      64
    #define DRV_MPU60X0_S_PWR_MGMT_1_CYCLE                      32
    #define DRV_MPU60X0_S_PWR_MGMT_1_TEMP_DIS                   8
    #define DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_CLOCK_STOP          7
    #define DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_PLL_EXT_19KHZ_REF   5
    #define DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_PLL_EXT_32KHZ_REF   4
    #define DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_PLL_Z_GYRO_REF      3
    #define DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_PLL_Y_GYRO_REF      2
    #define DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_PLL_X_GYRO_REF      1
    #define DRV_MPU60X0_S_PWR_MGMT_1_CLKSEL_INT_8MHZ            0

    // CONFIG SETTINGS
    #define DRV_MPU60X0_S_DLPF_CFG_260HZ_1MS  0
    #define DRV_MPU60X0_S_DLPF_CFG_185HZ_2MS  1
    #define DRV_MPU60X0_S_DLPF_CFG_96HZ_3MS   2
    #define DRV_MPU60X0_S_DLPF_CFG_43HZ_5MS   3 
    #define DRV_MPU60X0_S_DLPF_CFG_20HZ_8MS   4
    #define DRV_MPU60X0_S_DLPF_CFG_10HZ_14MS  5 
    #define DRV_MPU60X0_S_DLPF_CFG_5HZ_19MS   6

    #define DRV_MPU60X0_S_EXT_SYNC_INPUT_DISABLED 0
    #define DRV_MPU60X0_S_EXT_SYNC_TEMP_OUT       8       
    #define DRV_MPU60X0_S_EXT_SYNC_GYRO_XOUT_L    16
    #define DRV_MPU60X0_S_EXT_SYNC_GYRO_YOUT_L    24
    #define DRV_MPU60X0_S_EXT_SYNC_GYRO_ZOUT_L    32
    #define DRV_MPU60X0_S_EXT_SYNC_ACCEL_XOUT_L   40
    #define DRV_MPU60X0_S_EXT_SYNC_ACCEL_YOUT_L   48
    #define DRV_MPU60X0_S_EXT_SYNC_ACCEL_ZOUT_L   56

    // GYRO SETTINGS
    #define DRV_MPU60X0_S_FS_SEL_250    0
    #define DRV_MPU60X0_S_FS_SEL_500    8
    #define DRV_MPU60X0_S_FS_SEL_1000   16
    #define DRV_MPU60X0_S_FS_SEL_2000   24

    // ACCEL SETTINGS
    #define DRV_MPU60X0_S_AFS_SEL_2G     0
    #define DRV_MPU60X0_S_AFS_SEL_4G     8
    #define DRV_MPU60X0_S_AFS_SEL_8G     16
    #define DRV_MPU60X0_S_AFS_SEL_16G    24

 /******************************************************************************
 * Macros
 ******************************************************************************/
    #define DRV_MPU60X0_SET_NULL 0

 /*****************************************************************************
 * Typedefs and structures
 *****************************************************************************/

 /*****************************************************************************
 * Variables
 ******************************************************************************/

 /******************************************************************************
 * Function Definitions/Prototypes for PUBLIC functions
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

GenUtl_Err_t  Drv_MPU60X0Cfg_Init(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DRVGY_MPU6000_CFG_DRVGY_MPU6000_CFG_H_ */
