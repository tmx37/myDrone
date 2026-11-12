#include "TestSuite.h"
#include "stm32g0xx_hal.h"

#include "Drv_HwSkyWalker_Esc.h"
#include "Drv_BMP180.h"
#include "Drv_HMC5883L.h"
#include "Drv_MPU60X0.h"

const uint8_t* line_0 = " ";
const uint8_t* new_line = "\r\n";
const uint8_t* line_1 = "Raw_Pres: ";
const uint8_t* line_2 = "Pressione: ";
const uint8_t* line_3 = "[Pa] ";
const uint8_t* line_4 = "| Altitude: ";
const uint8_t* line_5 = "[m] ";
const uint8_t* line_6 = "| Temperatura: ";
const uint8_t* line_7 = " -> ";
const uint8_t* line_8 = "[C]";
const uint8_t* GY_X = "GY_X: ";
const uint8_t* GY_Y = "GY_Y: ";
const uint8_t* GY_Z = "GY_Z: ";
const uint8_t* ACC_X = "ACC_X: ";
const uint8_t* ACC_Y = "ACC_Y: ";
const uint8_t* ACC_Z = "ACC_Z: ";
char output_sting_final_temp[20];
char output_sting_final_pres[20];
char output_sting_altitude[20];
char output_sting_final_gx[20];
char output_sting_final_gy[20];
char output_sting_final_gz[20];
char output_sting_final_ax[20];
char output_sting_final_ay[20];
char output_sting_final_az[20];
char output_sting_final_mx[20];
char output_sting_final_my[20];
char output_sting_final_mz[20];

GY_Data_t GYOutputData;
ACC_Data_t ACCELOutputData;

GenUtl_Err_t TestSuite_Init()
{

    if(DrvGYBMP180Cfg_Init() != OUT_OK)
	  HAL_UART_Transmit(&huart2, "Can't start BMP180 driver!", strlen("Can't start BMP180 driver!"), 1000);

    if (DrvHMC5883LCfg_Init() != OUT_OK)
  	  HAL_UART_Transmit(&huart2, "Can't start HMC5883L driver!", strlen("Can't start HMC5883L driver!"), 1000);
 
    if (DrvMPU60X0Cfg_Init() != OUT_OK)
  	  HAL_UART_Transmit(&huart2, "Can't start MPU60X0 driver!", strlen("Can't start MPU60X0 driver!"), 1000);
  
    return OUT_OK;
}

GenUtl_Err_t TestSuite_Read_Hmc5883l()
{
    HMC_Make_Measurement_4_Single();
    HAL_Delay(6);
    HMC_Read_All(&compass);
    HAL_Delay(1000);

    itoa(compass.compass_x, output_sting_final_mx, 10);
    itoa(compass.compass_y, output_sting_final_my, 10);
    itoa(compass.compass_z, output_sting_final_mz, 10);

    HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_mx, strlen(output_sting_final_mx), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, " ", strlen(" "), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_my, strlen(output_sting_final_my), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, " ", strlen(" "), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_mz, strlen(output_sting_final_mz), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, "\r\n", strlen("\r\n"), HAL_MAX_DELAY);

    return OUT_OK;
}

GenUtl_Err_t TestSuite_Read_Bmp180()
{
    itoa(getTemperature(), output_sting_final_temp, 10);
	itoa(getPressure(), output_sting_final_pres, 10);
	itoa(getAltitude(), output_sting_altitude, 10);

	HAL_UART_Transmit(&huart2, line_2, strlen(line_2), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_pres, strlen(output_sting_final_pres), 1000);
	HAL_UART_Transmit(&huart2, line_3, strlen(line_3), 1000);
	HAL_UART_Transmit(&huart2, line_4, strlen(line_4), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_altitude, strlen(output_sting_altitude), 1000);
	HAL_UART_Transmit(&huart2, line_5, strlen(line_5), 1000);
	HAL_UART_Transmit(&huart2, line_6, strlen(line_6), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_temp, strlen(output_sting_final_temp), 1000);
	HAL_UART_Transmit(&huart2, line_8, strlen(line_8), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*)new_line, strlen(new_line), 1000);
    HAL_UART_Transmit(&huart2, line_2, strlen(line_2), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_pres, strlen(output_sting_final_pres), 1000);
	HAL_UART_Transmit(&huart2, line_3, strlen(line_3), 1000);

    return OUT_OK;
}

GenUtl_Err_t TestSuite_Read_Mpu60x0()
{
    if(getGyro(&GYOutputData) == OUT_OK)
    {       
        itoa(GYOutputData.XG, output_sting_final_gx, 10);
        HAL_UART_Transmit(&huart2, GY_X, strlen(GY_X), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_gx, strlen(output_sting_final_gx), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);

        itoa(GYOutputData.YG, output_sting_final_gy, 10);
        HAL_UART_Transmit(&huart2, GY_Y, strlen(GY_Y), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_gy, strlen(output_sting_final_gy), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
      
        itoa(GYOutputData.ZG, output_sting_final_gz, 10);
        HAL_UART_Transmit(&huart2, GY_Z, strlen(GY_Z), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_gz, strlen(output_sting_final_gz), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
    }
    else
    {
        HAL_UART_Transmit(&huart2, GY_Z, strlen(GY_Z), 1000);
        HAL_UART_Transmit(&huart2, "Can't get GY!!!", strlen("Can't get GY!!!"), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
    }
    
    if (getAccel(&ACCELOutputData) == OUT_OK)
    {
        itoa(ACCELOutputData.XA, output_sting_final_ax, 10);
        HAL_UART_Transmit(&huart2, ACC_X, strlen(ACC_X), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_ax, strlen(output_sting_final_ax), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);

        itoa(ACCELOutputData.YA, output_sting_final_ay, 10);
        HAL_UART_Transmit(&huart2, ACC_Y, strlen(ACC_Y), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_ay, strlen(output_sting_final_ay), 1000);
        HAL_UART_Transmit(&huart2, line_0, strlen(line_0), 1000);
      
        itoa(ACCELOutputData.ZA, output_sting_final_az, 10);
        HAL_UART_Transmit(&huart2, ACC_Z, strlen(ACC_Z), 1000);
        HAL_UART_Transmit(&huart2, (uint8_t*)output_sting_final_az, strlen(output_sting_final_az), 1000);
        HAL_UART_Transmit(&huart2, new_line, strlen(new_line), 1000);
    }
    else
    {
        HAL_UART_Transmit(&huart2, GY_Z, strlen(GY_Z), 1000);
        HAL_UART_Transmit(&huart2, "Can't get ACCEL!!!", strlen("Can't get ACCEL!!!"), 1000);
        HAL_UART_Transmit(&huart2, new_line, strlen(new_line), 1000);
    }

    return OUT_OK;
}

GenUtl_Err_t TestSuite_AppBolla_Mpu60x0()
{
    bool test_bolla;
    isDeviceOnPlainSurface(&test_bolla);
    HAL_UART_Transmit(&huart2, "Is the device in bolla? ", strlen("Is the device in bolla? "), 1000);
    if (test_bolla)
        HAL_UART_Transmit(&huart2, "CIANO XE IN BOLLA!", strlen("CIANO XE IN BOLLA!"), 1000);
    else
        HAL_UART_Transmit(&huart2, "NO A XE IN BOLLA..", strlen("NO A XE IN BOLLA.."), 1000);
    HAL_UART_Transmit(&huart2, new_line, strlen(new_line), 1000);

    return OUT_OK;
}

GenUtl_Err_t TestSuite_AppBussola_Hmc5883l()
{
    uint8_t CRA_config = 0x78;  // Configuration register A, defaults to 0x10 (00010000)
    uint8_t CRB_config = 0xA0;  // Configuration register B, defaults to 0x20 (00100000)
    uint8_t MODE_config = 0x00; // Selects measurement mode

    if (HAL_I2C_IsDeviceReady(&hi2c1, HMC5883L_ADDR_READ, 5, HAL_MAX_DELAY) == HAL_OK)
    {
        HAL_I2C_Mem_Write(&hi2c1, HMC5883L_DEVICE_ADDR, HMC5883L_CONFIG_A, 1, &CRA_config, 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Write(&hi2c1, HMC5883L_DEVICE_ADDR, HMC5883L_CONFIG_B, 1, &CRB_config, 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Write(&hi2c1, HMC5883L_DEVICE_ADDR, HMC5883L_MODE, 1, &MODE_config, 1, HAL_MAX_DELAY);
    } 

    uint8_t X_out_raw[2], Y_out_raw[2], Z_out_raw[2];
    uint16_t X_out_full, Y_out_full, Z_out_full = 0;

    uint8_t HMC_STATE = 0;
    hmc_t compass;
    HMC5883L_Init();
    
    return OUT_OK;
}