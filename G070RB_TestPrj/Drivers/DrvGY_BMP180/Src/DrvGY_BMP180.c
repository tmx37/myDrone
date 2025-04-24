/*
 * DrvGY_BMP180.c
 *
 *  Created on: Mar 19, 2025
 *      Author: tmx37
 */

#include "DrvGY_BMP180.h"

/******************************************************************************
 * Module Preprocessor Constants
 ******************************************************************************/

#define BMP180_WRITE 	0xEE
#define BMP180_READ 	0xEF
#define CONTROL_REG		0xF4
#define CALLIB_START 	0xAA

/******************************************************************************
 * Module Preprocessor Macros
 ******************************************************************************/

/******************************************************************************
 * Typedefs and structures
 *******************************************************************************/

/******************************************************************************
 * Module Variable Definitions
 ******************************************************************************/

static DrvGY_BMP180_WriteRegistersCB_t 	mWriteRegistersCB;
static DrvGY_BMP180_ReadRegisterCB_t 	mReadRegistersCB;
static DrvGY_BMP180_IsDeviceReadyCB_t	mIsDeviceReadyCB;

static DrvGYBMP180_TemperatureUnit 	mTemperatureUnit;
static DrvGYBMP180_AltitudeUnit 	mAltitudeUnit;
static DrvGYBMP180_PressureUnit		mPressureUnit;

static uint16_t 	AC1, AC4, AC5, AC6, B1, MD 	= 	0;
static int16_t 	AC2, AC3, MB, MC			= 	0;
static uint8_t		B2, oss						= 	0;

static uint8_t temp_45ms_control_data 	= 	0x2E;			// control_byte to read temperature
static uint8_t pres_45ms_control_data 	= 	0x34;			// control_byte to read temperature
static uint8_t out_msb_reg_addr 		= 	0xF6;			//out_msb register address, out_lsb and out_xlsb are subsequent to this memory position

static int32_t Sea_level_pressure_Pa = 101325;

static int32_t tX1_BMP180, tX2_BMP180, tX3_BMP180, tB5_BMP180, tB3_BMP180, tB6_BMP180 = 0;
static uint32_t tB4_BMP180, tB7_BMP180 = 0;

static uint8_t Temp_DataRAW[2], Pres_DataRAW[3] = {0};

static int32_t Final_Temp, Final_Pres, Altitude = 0;

static int32_t Raw_Temp, Raw_Pres = 0;

static uint8_t FailedTransmission = 0;

/******************************************************************************
 * Function Prototypes Private
 ******************************************************************************/

/******************************************************************************
 * Function Definitions Private
 ******************************************************************************/
UtlGen_Err_t BMP180_Calibration(void)
{
	uint8_t Callib_Data[22] = {0};

	mReadRegistersCB(0, BMP180_READ, CALLIB_START, Callib_Data, 22);

	if(!sizeof(Callib_Data)) return UTLGEN_ERR_NOT_FOUND;

	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);

	return UTLGEN_OK;
}

/******************************************************************************
 * Function Definitions Public
 ******************************************************************************/

UtlGen_Err_t DrvGY_BMP180_Init(const DrvGYBMP180_Config_t *pConfigData)
{
	mWriteRegistersCB = pConfigData->pfWriteRegistersI2cCB;
	mReadRegistersCB = pConfigData->pfReadRegisterI2cCB;
	mIsDeviceReadyCB = pConfigData->pfIsDeviceReadyI2cCB;

	oss = pConfigData->oversampling_setting;
	pres_45ms_control_data = 0x34+(oss<<6);

	mTemperatureUnit = CELSIUS;
	mPressureUnit = PASCAL;
	mAltitudeUnit = METER;

	if(isDeviceReady() != UTLGEN_OK) return UTLGEN_ERR_BUS_CONTENTION;

	if(BMP180_Calibration() != UTLGEN_OK) return UTLGEN_ERR_NOT_READY;

	return UTLGEN_OK;
}

int32_t getTemperature()
{
	/**
	 * bmp180_get_ut
	 */
	mWriteRegistersCB(0, BMP180_WRITE, CONTROL_REG, &temp_45ms_control_data, 1); // avvio al registro di controllo il valore per accedere alla temperatura salvata in EEPROM
	HAL_Delay(5); //4,5 ms for device ready-to-trasmit state, by doc
	mReadRegistersCB(0, BMP180_READ, out_msb_reg_addr, Temp_DataRAW, 2); // leso la temperatura al registro "out_msb" e "out_lsb": insieme fanno la temperatura raw

	Raw_Temp = (Temp_DataRAW[0]<< 8) +  Temp_DataRAW[1]; // viene restituio a 2 valori quindi devo convertire in un valore uint16 univoco

	tX1_BMP180 = ((Raw_Temp-AC6)* (AC5/pow(2, 15)));
	tX2_BMP180 = ((MC * (pow(2, 11))) / (tX1_BMP180 + MD));
	tB5_BMP180 = tX1_BMP180 + tX2_BMP180;
	Final_Temp = (uint16_t)(((tB5_BMP180 + 8) / pow(2, 4))/10);

	return Final_Temp;
}

int32_t getPressure()
{
	// Necessario per calcolo della Pressione. Valutare di farlo ogni tot tick di timer interno STM32 per non intasare di operazioni il dispositivo.
	getTemperature();

	/**
	 * bmp180_get_up
	 */
	mWriteRegistersCB(0, BMP180_WRITE, CONTROL_REG, &pres_45ms_control_data, 1);
	switch (oss)
	{
		case (0):
				HAL_Delay (5);
				break;
		case (1):
				HAL_Delay (8);
				break;
		case (2):
				HAL_Delay (14);
				break;
		case (3):
				HAL_Delay (26);
				break;
	}
	mReadRegistersCB(0, BMP180_READ, out_msb_reg_addr, Pres_DataRAW, 3);

	Raw_Pres = ((Pres_DataRAW[0] << 16) + (Pres_DataRAW[1] << 8) + Pres_DataRAW[2]) >> (8 - oss);

	tX1_BMP180 = ((Raw_Temp-AC6) * (AC5 / (pow(2,15))));
	tX2_BMP180 = ((MC * (pow(2,11))) / ( tX1_BMP180 + MD ));
	tB5_BMP180 = tX1_BMP180 + tX2_BMP180;
	tB6_BMP180 = tB5_BMP180 - 4000;
	tX1_BMP180 = (B2 * (tB6_BMP180 * tB6_BMP180 / (pow(2, 12)))) / (pow(2, 11));
	tX2_BMP180 = AC2 * tB6_BMP180 / pow(2, 11);
	tX3_BMP180 = tX1_BMP180 + tX2_BMP180;
	tB3_BMP180 = (((AC1 * 4 + tX3_BMP180)<<oss) + 2) / 4;
	tX1_BMP180 = AC3 * tB6_BMP180 / pow(2, 13);
	tX2_BMP180 = (B1 * (tB6_BMP180 * tB6_BMP180 / (pow(2, 12)))) / (pow(2, 16));
	tX3_BMP180 = ((tX1_BMP180 + tX2_BMP180) + 2) / pow(2, 2);
	tB4_BMP180 = AC4 * (uint32_t)(tX3_BMP180 + 32768) / (pow(2, 15));
	tB7_BMP180 = ((uint32_t)Raw_Pres - tB3_BMP180) * (50000>>oss);

	Final_Pres = (tB7_BMP180 < 0x80000000) ? ((tB7_BMP180 * 2) / tB4_BMP180) : ((tB7_BMP180/tB4_BMP180) * 2);
	tX1_BMP180 = (Final_Pres / (pow(2, 8))) * (Final_Pres / (pow(2, 8)));
	tX1_BMP180 = (tX1_BMP180 * 3038) / (pow(2, 16));
	tX2_BMP180 = (-7357 * Final_Pres) / (pow(2, 16));

	Final_Pres = Final_Pres + (tX1_BMP180 + tX2_BMP180 + 3791) / (pow(2, 4));

	return Final_Pres;
}

int32_t getAltitude()
{
	if(Final_Pres <= 0) getPressure();
	double temp = (double)Final_Pres/(double)Sea_level_pressure_Pa;
	Altitude = 44330*(1-(pow(temp, 0.19029495718)));
	return Altitude;
}

UtlGen_Err_t isDeviceReady()
{
	if (!mIsDeviceReadyCB(0, BMP180_WRITE, 5) || !mIsDeviceReadyCB(0, BMP180_READ, 5))
		return UTLGEN_OK;
	else
	{
		if(FailedTransmission >= 10)
			return UTLGEN_ERR_BUS_CONTENTION;
		FailedTransmission += 1;
		isDeviceReady();
	}
	return UTLGEN_ERR_BUSY;
}

void setTemperatureUnit(DrvGYBMP180_TemperatureUnit unit)
{
	mTemperatureUnit = unit;
}

void setPressureUnit(DrvGYBMP180_PressureUnit unit)
{
	mPressureUnit = unit;
}

void setAltitudeUnit(DrvGYBMP180_AltitudeUnit unit)
{
	mAltitudeUnit = unit;
}

int32_t convTemp(uint8_t value, DrvGYBMP180_TemperatureUnit unit)
{
	switch (mTemperatureUnit){
	case KELVIN:
		if(unit == CELSIUS) return ( value - 273 );
		else if(unit == FAHRENHEIT) return ( 1.8 * (value - 273.15) + 32 );
		break;
	case CELSIUS:
		if(unit == KELVIN) return ( value + 273 );
		else if(unit == FAHRENHEIT) return (( value * 1.8 ) + 32 );
		break;
	case FAHRENHEIT:
		if(unit == CELSIUS) return ((( value - 32 ) * 5 ) / 9 );
		else if(unit == KELVIN) return ( ( value + 460 ) * ( 5 / 9 ) );
		break;
	}
	return value;
}

int32_t convPres(uint8_t value, DrvGYBMP180_PressureUnit unit)
{
	switch (mPressureUnit){
	case PASCAL:
		if(unit == HPASCAL) return ( value / 100 );
		break;
	case HPASCAL:
		if(unit == PASCAL) return ( value * 100 );
		break;
	}
	return value;
}

int32_t convAltitude(uint8_t value, DrvGYBMP180_AltitudeUnit unit)
{
	switch (mAltitudeUnit){
	case METER:
		if(unit == FEET) return ( value * 3.3 );
		break;
	case FEET:
		if(unit == METER) return ( value / 3.3 );
		break;
	}
	return value;
}

/*************** END OF FUNCTIONS *********************************************/
