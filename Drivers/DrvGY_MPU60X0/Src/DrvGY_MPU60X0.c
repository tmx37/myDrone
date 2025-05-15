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

// TODO: find another solution to this
#define MPU60X0_ADDR 0x0068

#define MPU60X0_SAMPLE_RATE_DIV 0x19

#define MPU60X0_CONFIG 0x1A

#define MPU60X0_SMPRT_DIV 0x25

#define MPU60X0_GYRO_CONFIG 0x1B
#define MPU60X0_ACCEL_CONFIG 0x1C

#define MPU60X0_ACCEL_X_OUT_H 0x3B
#define MPU60X0_ACCEL_X_OUT_L 0x3C
#define MPU60X0_ACCEL_Y_OUT_H 0x3D
#define MPU60X0_ACCEL_Y_OUT_L 0x3E
#define MPU60X0_ACCEL_Z_OUT_H 0x3F
#define MPU60X0_ACCEL_Z_OUT_L 0x40

#define MPU60X0_GYRO_X_OUT_H 0x43
#define MPU60X0_GYRO_X_OUT_L 0x44
#define MPU60X0_GYRO_Y_OUT_H 0x45
#define MPU60X0_GYRO_Y_OUT_L 0x46
#define MPU60X0_GYRO_Z_OUT_H 0x47
#define MPU60X0_GYRO_Z_OUT_L 0x48

#define MPU60X0_SIGNAL_PATH_RESET 0x68
#define GYRO_RESET_SIGNAL 4
#define ACCEL_RESET_SIGNAL 2

#define MPU60X0_PWE_MGMT_1 0x6B
#define MPU60X0_PWE_MGMT_2 0x6C

#define MPU60X0_WHO_AM_I 0x75

/******************************************************************************
 * Module Variable Definitions
 ******************************************************************************/

 static DrvGY_MPU60X0_WriteRegisterCB_t mWriteRegistersCB;
 static DrvGY_MPU60X0_ReadRegisterCB_t mReadRegistersCB;
 static DrvGY_MPU60X0_IsDeviceReadyCB_t mIsDeviceReadyCB;
 static DrvGY_MPU60X0_MasterTransmitCB_t mMasterTrasmitCB;
 static DrvGY_MPU60X0_MasterReceiveCB_t mMasterReceiveCB;
 static DrvGY_MPU60X0_DelayCB_t mDelayCB;

 static DrvMPU60X0_Settings_t mSettings;

 static XYZ_Angles_t CurrentAngle = { 0, 0, 0 };

 static uint8_t gx_out_addr_h = MPU60X0_GYRO_X_OUT_H; 
 static uint8_t gy_out_addr_h = MPU60X0_GYRO_Y_OUT_H; 
 static uint8_t gz_out_addr_h = MPU60X0_GYRO_Z_OUT_H; 

 static uint8_t ax_out_addr_h = MPU60X0_ACCEL_X_OUT_H;
 static uint8_t ay_out_addr_h = MPU60X0_ACCEL_Y_OUT_H; 
 static uint8_t az_out_addr_h = MPU60X0_ACCEL_Z_OUT_H; 

 static uint8_t XGRaw[2], YGRaw[2], ZGRaw[2] = { 0 };
 static uint8_t XARaw[2], YARaw[2], ZARaw[2] = { 0 };

 static int t_LSB_Gyro_Sensitivity, t_LSB_Accel_Sensitivity = 0;

/******************************************************************************
 * Function Definitions Private
 ******************************************************************************/

 void SelectCorrectLSB();

UtlGen_Err_t DrvGY_MPU60X0_Init(const DrvMPU60X0_Config_t *pConfigData, const DrvMPU60X0_Settings_t *pSettingsData)
{
    mIsDeviceReadyCB = pConfigData->pfIsDeviceReadyI2cCB;
    mMasterReceiveCB = pConfigData->pfMasterReceiveCB;
    mMasterTrasmitCB = pConfigData->pfMasterTransmitCB;
    mReadRegistersCB = pConfigData->pfReadRegisterI2cCB;
    mWriteRegistersCB = pConfigData->pfWriteRegistersI2cCB;
    mDelayCB = pConfigData->pfDelayCB;

    if (pConfigData == NULL || pSettingsData == NULL)
        return UTLGEN_ERR_INVALID_ARG;

    mSettings = *pSettingsData;

    SelectCorrectLSB();
    
    if (!mIsDeviceReadyCB(0, MPU60X0_ADDR, 5))
        return UTLGEN_ERR_NOT_READY;

    // PowerUp Reset
    uint8_t Pwr_Mgmt_Settings_Reset[2] = { MPU60X0_PWE_MGMT_1, mSettings.RESET_STARTUP };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, Pwr_Mgmt_Settings_Reset, 2);
    mDelayCB(110);

    // PowerUp configuration
    uint8_t Pwr_Mgmt_Settings_CustomStartup[2] = { MPU60X0_PWE_MGMT_1, mSettings.PWE_MGMT_1_SETTINGS };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, Pwr_Mgmt_Settings_CustomStartup, 2);

    // DLPF(Digital Low Pass Filter) and FSYNC (Frame Syncronization) pin sampling configuration
    uint8_t DLPF_FSYNC_Config_Settings[2] = { MPU60X0_CONFIG, mSettings.DLPF_FSYNC_SETTINGS };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, DLPF_FSYNC_Config_Settings, 2);

    // Sample rate configuration (SMPRT_DIV)
    // uint8_t SampleRateDivider = 0;
    // mWriteRegistersCB(0, MPU60X0_ADDR << 1, MPU60X0_SAMPLE_RATE_DIV, &SampleRateDivider, 1);

    // Gyroscope configuration
    uint8_t GYRO_Config_Settings[2] = { MPU60X0_GYRO_CONFIG, mSettings.GYRO_SETTINGS };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, GYRO_Config_Settings, 2);

    // Accellerometer configuration
    uint8_t ACCEL_Config_Settings[2] = { MPU60X0_ACCEL_CONFIG, mSettings.ACCEL_SETTINGS };
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, ACCEL_Config_Settings, 2);

    // CurrentAngle configuration
    // TODO: Do "CurrentAngle" set to 0 after accellerometer check
    // > Try to get a more precise accellerometer mesaurament before

    return UTLGEN_OK;
}

UtlGen_Err_t DrvGY_MPU60X0_GyroSignalPathReset()
{
    if (!mIsDeviceReady(0, MPU60X0_ADDR, 5))
        return UTLGEN_ERR_NOT_READY;
    
    if(mWriteRegistersCB(0, MPU60X0_ADDR << 1, MPU60X0_SIGNAL_PATH_RESET, GYRO_RESET_SIGNAL, 1) == UTLGEN_OK)
        return UTLGEN_OK;
    else 
        return UTLGEN_ERR_NOT_SUPPORTED;
}

UtlGen_Err_t DrvGY_MPU60X0_AccelSignalPathReset()
{
    if (!mIsDeviceReady(0, MPU60X0_ADDR, 5))
        return UTLGEN_ERR_NOT_READY;
        
    if(mWriteRegistersCB(0, MPU60X0_ADDR << 1, MPU60X0_SIGNAL_PATH_RESET, ACCEL_RESET_SIGNAL, 1) == UTLGEN_OK)
        return UTLGEN_OK;
    else 
        return UTLGEN_ERR_NOT_SUPPORTED;
}

/*
    TODO: 
    > will probably deprecate once getAngles() and isDeviceOnPlainSurface() will be moved to the upper layer
*/
UtlGen_Err_t setSamplingTime(uint8_t mseconds)
{
    if (mseconds <= 0) 
        return UTLGEN_ERR_INVALID_ARG;

    return UTLGEN_OK;
}

UtlGen_Err_t getGyro(GY_Data_t *output)
{
    if (output == NULL) 
        return UTLGEN_ERR_INVALID_ARG;

    if (!mIsDeviceReadyCB(0, MPU60X0_ADDR, 5))
        return UTLGEN_ERR_NOT_READY;

    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, &gx_out_addr_h, 1);
    mMasterReceiveCB(0, MPU60X0_ADDR << 1, &XGRaw[0], 2);
    
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, &gy_out_addr_h, 1);
    mMasterReceiveCB(0, MPU60X0_ADDR << 1, &YGRaw[0], 2);
    
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, &gz_out_addr_h, 1);
    mMasterReceiveCB(0, MPU60X0_ADDR << 1, &ZGRaw[0], 2);

    output->XG = (XGRaw[0] << 8) + XGRaw[1];
    output->YG = (YGRaw[0] << 8) + YGRaw[1];
    output->ZG = (ZGRaw[0] << 8) + ZGRaw[1];
    
    if (output->XG >= 32767) output->XG = output->XG - 65536;
    if (output->YG >= 32767) output->YG = output->YG - 65536;
    if (output->ZG >= 32767) output->ZG = output->ZG - 65536;

    // Scale raw value based on sensitivity (eg: +- 250 deg/s = 131 LSB/°/s -> dipende dal valore di FS_SEL (0x1B))
    output->XG = output->XG / t_LSB_Gyro_Sensitivity;
    output->YG = output->YG / t_LSB_Gyro_Sensitivity;
    output->ZG = output->ZG / t_LSB_Gyro_Sensitivity;

    return UTLGEN_OK;
}

UtlGen_Err_t getAccel(ACC_Data_t *output)
{
    if (output == NULL) 
        return UTLGEN_ERR_INVALID_ARG;

    if (!mIsDeviceReadyCB(0, MPU60X0_ADDR, 5))
        return UTLGEN_ERR_NOT_READY;

    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, &ax_out_addr_h, 1);
    mMasterReceiveCB(0, MPU60X0_ADDR << 1, &XARaw[0], 2);
    
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, &ay_out_addr_h, 1);
    mMasterReceiveCB(0, MPU60X0_ADDR << 1, &YARaw[0], 2);
    
    mMasterTrasmitCB(0, MPU60X0_ADDR << 1, &az_out_addr_h, 1);
    mMasterReceiveCB(0, MPU60X0_ADDR << 1, &ZARaw[0], 2);

    output->XA = (XARaw[0] << 8) + XARaw[1];
    output->YA = (YARaw[0] << 8) + YARaw[1];
    output->ZA = (ZARaw[0] << 8) + ZARaw[1];
    
    if (output->XA >= 32767) output->XA = output->XA - 65536;
    if (output->YA >= 32767) output->YA = output->YA - 65536;
    if (output->ZA >= 32767) output->ZA = output->ZA - 65536;

    // Scale raw value based on sensitivity (eg: +- 2g = 16384 LSB/g -> dipende dal valore di AFS_SEL (0x1C))
    // output->XA = output->XA / t_LSB_Accel_Sensitivity;
    // output->YA = output->YA / t_LSB_Accel_Sensitivity;
    // output->ZA = output->ZA / t_LSB_Accel_Sensitivity;

    return UTLGEN_OK;
} 

/*
    TODO: 
    > develop here but it will be probably moved on a superior layer (maybe in a control routine in the main FreeRTOS);
    > Set the base angles only when isDeviceOnPlainSurface returns true;
    > read gyro and calculate the angle variation with a precise punderate media of time taken to read the measurement. 
*/
UtlGen_Err_t getAngles(XYZ_Angles_t *output)
{
    if (output == NULL) 
        return UTLGEN_ERR_INVALID_ARG;

    return UTLGEN_OK;
}

/*
    TODO: 
    > develop here but it will be probably moved on a superior layer (maybe in a control routine in the main FreeRTOS)
    > read accellerometer values and return true only if we have an approximate 1g accelleration (~= 9.81m/s2) on Z axis and and acceptable value on X and Y
    > dont know if this will be harmed by aerial movements, maybe it should only be performed once drone is "stable" (analyze vibration and so on)
*/
UtlGen_Err_t isDeviceOnPlainSurface(bool *output)
{
    if (output == NULL) 
        return UTLGEN_ERR_INVALID_ARG;
        
    ACC_Data_t t_acc_data;
    if (getAccel(&t_acc_data) != UTLGEN_OK)
        return UTLGEN_ERR_NOT_FOUND;

    uint8_t t_anti_err_counter = 0; 

    if (t_acc_data.ZA >= (t_LSB_Accel_Sensitivity - 100) && t_anti_err_counter < 8)
    {
        *output = true;
        t_anti_err_counter = 0;
    }    
    else
    {
        *output = false;
        t_anti_err_counter++;
    }

    return UTLGEN_OK;
}

UtlGen_Err_t isMPU60X0Here(bool *output)
{
    if (output == NULL) 
        return UTLGEN_ERR_INVALID_ARG;

    uint8_t outputBuffer;
    if(mReadRegistersCB(0, MPU60X0_ADDR << 1, MPU60X0_WHO_AM_I, &outputBuffer, 1) != 0) 
        return UTLGEN_ERR_NOT_FOUND;
    
    *output = (outputBuffer) ? true : false;

    return UTLGEN_OK;
}


void SelectCorrectLSB()
{
    switch (mSettings.FS_SEL)
    {
        case 0:
            t_LSB_Gyro_Sensitivity = 131;
            break;
        case 8:
            t_LSB_Gyro_Sensitivity = 65;
            break;
        case 16:
            t_LSB_Gyro_Sensitivity = 32;
            break;
        case 24:
            t_LSB_Gyro_Sensitivity = 16;
            break;
        default:
            t_LSB_Gyro_Sensitivity = 131;
            break;
    }
    switch (mSettings.AFS_SEL)
    {
        case 0:
            t_LSB_Accel_Sensitivity = 16384;
            break;
        case 8:
            t_LSB_Accel_Sensitivity = 8192;
            break;
        case 16:
            t_LSB_Accel_Sensitivity = 4096;
            break;    
        case 24:
            t_LSB_Accel_Sensitivity = 2048;
            break;
        default:
            t_LSB_Accel_Sensitivity = 16384;
            break;
    }
}