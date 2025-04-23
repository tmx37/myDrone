/*
 * DrvGY_BMP180.h
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#ifndef DRVGY_BMP180_INC_DRVGY_BMP180_H_
#define DRVGY_BMP180_INC_DRVGY_BMP180_H_


/******************************************************************************
 * Includes
 *******************************************************************************/

#include "DrvGY_BMP180_Cfg.h"

/******************************************************************************
 * Module Typedefs
 ******************************************************************************/
typedef enum
{
	KELVIN,
	CELSIUS,
	FAHRENHEIT
} DrvGYBMP180_TemperatureUnit;

typedef enum
{
	PASCAL,
	HPASCAL
} DrvGYBMP180_PressureUnit;

typedef enum
{
	METER,
	FEET
} DrvGYBMP180_AltitudeUnit;

typedef uint32_t (*DrvGY_BMP180_WriteRegistersCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);

typedef uint32_t (*DrvGY_BMP180_ReadRegisterCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t RegNdx, uint8_t *pDataBuffer, uint32_t DataSize);

typedef uint32_t (*DrvGY_BMP180_IsDeviceReadyCB_t)(uint32_t I2cInst, uint8_t DevAddr, uint8_t nTrials);

/**
 * BMP180 configuration istance
 */
typedef struct
{
	uint8_t I2CInst;	// solo una istanza I2C dato che su GY-85 è sempre e solo una
	DrvGY_BMP180_WriteRegistersCB_t pfWriteRegistersI2cCB;
	DrvGY_BMP180_ReadRegisterCB_t pfReadRegisterI2cCB;
	DrvGY_BMP180_IsDeviceReadyCB_t pfIsDeviceReadyI2cCB;

	DrvGYBMP180_AltitudeUnit AltitudeUnit;
	DrvGYBMP180_PressureUnit PressureUnit;
	DrvGYBMP180_TemperatureUnit TemperatureUnit;

	uint8_t oversampling_setting;
} DrvGYBMP180_Config_t;

/******************************************************************************
 * Function Definitions/Prototypes for PUBLIC functions
 *******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

UtlGen_Err_t DrvGY_BMP180_Init(const DrvGYBMP180_Config_t *pConfigData);

int32_t getTemperature();

int32_t getPressure();

int32_t getAltitude();

UtlGen_Err_t isDeviceReady();

void setTemperatureUnit(DrvGYBMP180_TemperatureUnit unit);

void setPressureUnit(DrvGYBMP180_PressureUnit unit);

void setAltitudeUnit(DrvGYBMP180_AltitudeUnit unit);

int32_t convTemp(uint8_t value, DrvGYBMP180_TemperatureUnit unit);

int32_t convPres(uint8_t value, DrvGYBMP180_PressureUnit unit);

int32_t convAltitude(uint8_t value, DrvGYBMP180_AltitudeUnit unit);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DRVGY_BMP180_INC_DRVGY_BMP180_H_ */

/*************** END OF FUNCTIONS *********************************************/
