/**
 * GenUtl
 * 
 * Made for common error codes and general function utilities.
*/

#include <stdint.h>

typedef int32_t GenUtl_Err_t;

/**
 * @brief Common return codes
 */
#define OUT_OK                      (GenUtl_Err_t)(0)      /*!< Success (no error)                 */
#define OUT_FAIL                    (GenUtl_Err_t)(-1)     /*!< Generic failure                    */

#define OUT_ERR_NO_MEM              (GenUtl_Err_t)(-2)     /*!< Out of memory                      */
#define OUT_ERR_INVALID_ARG         (GenUtl_Err_t)(-3)     /*!< Invalid argument                   */
#define OUT_ERR_INVALID_STATE       (GenUtl_Err_t)(-4)     /*!< Invalid state                      */
#define OUT_ERR_INVALID_SIZE        (GenUtl_Err_t)(-5)     /*!< Invalid size                       */
#define OUT_ERR_NOT_FOUND           (GenUtl_Err_t)(-6)     /*!< Requested resource not found       */
#define OUT_ERR_NOT_SUPPORTED       (GenUtl_Err_t)(-7)     /*!< Operation or feature not supported */
#define OUT_ERR_TIMEOUT             (GenUtl_Err_t)(-8)     /*!< Operation timed out                */
#define OUT_ERR_INVALID_RESPONSE    (GenUtl_Err_t)(-9)     /*!< Received response was invalid      */
#define OUT_ERR_INVALID_CRC         (GenUtl_Err_t)(-10)    /*!< CRC or checksum was invalid        */
#define OUT_ERR_INVALID_VERSION     (GenUtl_Err_t)(-11)    /*!< Version was invalid                */
#define OUT_ERR_INVALID_MAC         (GenUtl_Err_t)(-12)    /*!< MAC address was invalid            */
#define OUT_ERR_BUSY                (GenUtl_Err_t)(-13)    /*!< Peripheral is busy                 */
#define OUT_ERR_NOT_READY           (GenUtl_Err_t)(-14)    /*!< Peripheral/source is not ready     */
#define OUT_ERR_BUS_CONTENTION      (GenUtl_Err_t)(-15)    /*!< Bus contention detected            */