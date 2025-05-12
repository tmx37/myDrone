/*
 * DrvGY_MPU6000.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#include "DrvGY_MPU60X0.h"

/******************************************************************************
 * Module Preprocessor Constants
 ******************************************************************************/

#define MPU6000_SAMPLE_RATE_DIV 0x19

#define MPU6000_CONFIG 0x1A

#define MPU6000_SMPRT_DIV 0x25

#define MPU6000_GYRO_CONFIG 0x27
#define MPU6000_ACCL_CONFIG 0x28

#define MPU6000_ACC_X_OUT_H 0x3B
#define MPU6000_ACC_X_OUT_L 0x3C
#define MPU6000_ACC_Y_OUT_H 0x3D
#define MPU6000_ACC_Y_OUT_L 0x3E
#define MPU6000_ACC_Z_OUT_H 0x3F
#define MPU6000_ACC_Z_OUT_L 0x40

#define MPU6000_GYRO_X_OUT_H 0x43
#define MPU6000_GYRO_X_OUT_L 0x44
#define MPU6000_GYRO_Y_OUT_H 0x45
#define MPU6000_GYRO_Y_OUT_L 0x46
#define MPU6000_GYRO_Z_OUT_H 0x47
#define MPU6000_GYRO_Z_OUT_L 0x48

#define MPU6000_SIGNAL_PATH_RESET 0x68

#define MPU6000_PWE_MGMT_1 0x6B
#define MPU6000_PWE_MGMT_2 0x6C

#define MPU6000_WHO_AM_I 0x75

/******************************************************************************
 * Module Variable Definitions
 ******************************************************************************/

 static DrvGY_MPU60X0_WriteRegisterCB_t mWriteRegistersCB;
 static DrvGY_MPU60X0_ReadRegisterCB_t mReadRegistersCB;
 static DrvGY_MPU60X0_IsDeviceReadyCB_t mIsDeviceReady;
 static DrvGY_MPU60X0_MasterTransmitCB_t mMasterTrasmitCB;
 static DrvGY_MPU60X0_MasterReceiveCB_t mMasterReceiveCB;
 static DrvGY_MPU60X0_DelayCB_t mDelayCB;

 static XYZAngles CurrentAngle = { 0, 0, 0};

/******************************************************************************
 * Function Definitions Private
 ******************************************************************************/

UtlGen_Err_t DrvGY_MPU60X0_Init(const DrvMPU60X0_Config_t *pConfigData)
{

    uint8_t Pwr_Mgmt_ResetNSleep[] = { 0x6B, 0xC0 };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, Pwr_Mgmt_ResetNSleep, 2);
    mDelayCB(110);
    

    return UTLGEN_OK;
}

// TODO: UtlGen_Err setSamplingTime(uint8_t mseconds);
UtlGen_Err_t setSamplingTime(uint8_t mseconds)
{
    return UTLGEN_OK;
}

// TODO: UtlGen_Err getGyro(&GYData output); 
UtlGen_Err_t getGyro()
{
    return UTLGEN_OK;
}

// TODO: UtlGen_Err getAngles(&XYZAngles output); 
UtlGen_Err_t getAngles()
{
    return UTLGEN_OK;
}

// TODO: UtlGen_Err getAcc(&ACCData output); 
UtlGen_Err_t getAccel()
{
    return UTLGEN_OK;
} 