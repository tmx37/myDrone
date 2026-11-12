/**
 * TestSuite
 * 
 * Application made only for tests to execute during runtime.
 */

#include "GenUtl.h"

/**
 * @brief Test suite init of all modules
 */
GenUtl_Err_t TestSuite_Init();

/** Read tests */
GenUtl_Err_t TestSuite_Read_Hmc5883l();

GenUtl_Err_t TestSuite_Read_Bmp180();

GenUtl_Err_t TestSuite_Read_Mpu60x0();
/** */

/** Basic application tests */
GenUtl_Err_t TestSuite_AppBolla_Mpu60x0();

GenUtl_Err_t TestSuite_AppBussola_Hmc5883l();
/** */

