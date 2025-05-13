/*
 * DrvGY_MPU6000.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#include "DrvGY_MPU60X0.h"

#include <string.h>

/******************************************************************************
 * Module Preprocessor Constants
 ******************************************************************************/

#define MPU60X0_SAMPLE_RATE_DIV 0x19

#define MPU60X0_CONFIG 0x1A

#define MPU60X0_SMPRT_DIV 0x25

#define MPU60X0_GYRO_CONFIG 0x1B
#define MPU60X0_ACCEL_CONFIG 0x1C

#define MPU60X0_ACC_X_OUT_H 0x3B
#define MPU60X0_ACC_X_OUT_L 0x3C
#define MPU60X0_ACC_Y_OUT_H 0x3D
#define MPU60X0_ACC_Y_OUT_L 0x3E
#define MPU60X0_ACC_Z_OUT_H 0x3F
#define MPU60X0_ACC_Z_OUT_L 0x40

#define MPU60X0_GYRO_X_OUT_H 0x43
#define MPU60X0_GYRO_X_OUT_L 0x44
#define MPU60X0_GYRO_Y_OUT_H 0x45
#define MPU60X0_GYRO_Y_OUT_L 0x46
#define MPU60X0_GYRO_Z_OUT_H 0x47
#define MPU60X0_GYRO_Z_OUT_L 0x48

#define MPU60X0_SIGNAL_PATH_RESET 0x68

#define MPU60X0_PWE_MGMT_1 0x6B
#define MPU60X0_PWE_MGMT_2 0x6C

#define MPU60X0_WHO_AM_I 0x75

/******************************************************************************
 * Module Variable Definitions
 ******************************************************************************/

 static DrvGY_MPU60X0_WriteRegisterCB_t mWriteRegistersCB;
 static DrvGY_MPU60X0_ReadRegisterCB_t mReadRegistersCB;
 static DrvGY_MPU60X0_IsDeviceReadyCB_t mIsDeviceReady;
 static DrvGY_MPU60X0_MasterTransmitCB_t mMasterTrasmitCB;
 static DrvGY_MPU60X0_MasterReceiveCB_t mMasterReceiveCB;
 static DrvGY_MPU60X0_DelayCB_t mDelayCB;

 static XYZ_Angles CurrentAngle = { 0, 0, 0 };

/******************************************************************************
 * Function Definitions Private
 ******************************************************************************/

UtlGen_Err_t DrvGY_MPU60X0_Init(const DrvMPU60X0_Config_t *pConfigData)
{
    // PowerUp Reset
    uint8_t Pwr_Mgmt_Settings_Reset[2] = { MPU60X0_PWE_MGMT_1, 0xC0 };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, Pwr_Mgmt_Settings_Reset, 2);
    mDelayCB(110);

    // PowerUp configuration
    // TODO: Modify second value with custom configuration ( NB: disable temperature sensor by default )
    uint8_t Pwr_Mgmt_Settings_CustomStartup[2] = { MPU60X0_PWE_MGMT_1, 0x00 };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, Pwr_Mgmt_Settings_CustomStartup, 2);

    // DLPF(Digital Low Pass Filter) and FSYNC (Frame Syncronization) pin sampling configuration
    // TODO: Modify second value with custom configuration
    uint8_t DLPF_FSYNC_Config_Settings[2] = { MPU60X0_CONFIG, 4 };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, DLPF_FSYNC_Config_Settings, 2);

    // Gyroscope configuration
    // TODO: Modify second value with custom configuration
    uint8_t GYRO_Config_Settings[2] = { MPU60X0_GYRO_CONFIG, 0x00 };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, GYRO_Config_Settings, 2);

    // Accellerometer configuration
    // TODO: Modify second value with custom configuration
    uint8_t ACCEL_Config_Settings[2] = { MPU60X0_ACCEL_CONFIG, 0x00 };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, ACCEL_Config_Settings, 2);

    // CurrentAngle configuration
    // TODO: Do "CurrentAngle" set to 0 after accellerometer check
    // > Try to get a more precise accellerometer mesaurament before

    return UTLGEN_OK;
}

// TODO: ensure a precise timing is given to 
UtlGen_Err_t setSamplingTime(uint8_t mseconds)
{

    return UTLGEN_OK;
}

UtlGen_Err_t getGyro(GY_Data *output)
{

    return UTLGEN_OK;
}

UtlGen_Err_t getAngles(XYZ_Angles *output)
{

    return UTLGEN_OK;
}

UtlGen_Err_t getAccel(ACC_Data *output)
{

    return UTLGEN_OK;
} 