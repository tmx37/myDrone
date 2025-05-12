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

#define MPU6000_SLAVE_0 0x0068

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