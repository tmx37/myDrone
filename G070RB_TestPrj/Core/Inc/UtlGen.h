/****************************************************************************
 * Title                 :   Utilities library
 * Filename              :   UtlGen.h
 * Author                :
 * Origin Date           :   22/10/24
 * Version               :   2.0.0
 * Compiler              :   Generic
 * Target                :   Generic
 * Notes                 :
 *
 * Copyright (c) Vimar SpA. All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY VIMAR "AS IS" AND ANY EXPRESSED
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL VIMAR OR ITS CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/
 /******************************************************************************
  * Doxygen H Template
  *******************************************************************************/
  /*************** CHANGE LIST **************************************************
   *
   *  Date       Version     Author               Description
   *  01/01/22   0.1.0       Roberto Monti        Interface Created.
   *  22/03/22   0.1.3       Roberto Monti        added swap macros and CRC8 calc functions
   *  01/04/22   0.1.4       Francesco Alberti    Added Decrease Counter Custom functions
   *  04/04/22   0.1.5       Roberto Monti        added versions and UTLGEN_TO_STR macros
   *  08/04/22   0.2.0       Francesco Alberti    Version number updated. Changes for runtime log configuration and buffer dump
   *  29/04/22   0.3.0       Stefano Venditti     Random generator included
   *  17/05/22   0.4.0       Francesco Alberti    Added memory and software timer management. Added new macros (UTLGEN_BIT...)
   *  20/05/22   0.5.0       Belligio Alex        Added CRC16
   *  27/05/22   0.5.1       Francesco Alberti    Removed attribute INLINE and PUT in Ram.
   *                                              Moved attribute weak for random number in the CFG.
   *                                              Added UID_BASE define for ARMUUID Address
   *  16/06/22   0.6.0       Francesco Alberti    Added STM32 microcontroller utilities function.
   *  04/07/22   0.6.1       Francesco Alberti    Bug fixed on UtlTim_isInUse. UTLTIM_EXIT_CRITICAL was after the return.
   *  12/07/22   0.7.0       Stefano Venditti     Hamming code computation added
   *  13/07/22   0.7.1       Francesco Alberti    Fixed warning
   *  14/07/22   0.8.0       Stefano Venditti     Utl2fl module added
   *  25/07/22   0.8.1       Francesco Alberti    Bug fixed on function UtlTim_ChangePeriod, timer software. Added IsConfigured flag on Stm module.
   *  05/12/22   0.9.0       D.F.Granada          UtlDWT module added
   *  07/12/22   0.10.0      Francesco Alberti    Bug fixed on returned value of several UtlTim functions.
   *                                              Added UtlTim_TimerGet/SetPeriod functions.
   *                                              Added UTLGEN_UNUSED macro.
   *  15/12/22   0.10.1      Mirko Grillo         Fix casting
   *  15/12/22   0.11.0      Stefano Venditti     New ASSERT function
   *  19/12/22   0.11.1      Francesco Alberti    Bug fixed on handle timer (UtlTim) management.
   *  19/12/22   0.11.2      Mirko Grillo         Fix do while
   *  21/12/22   0.11.3      Francesco Alberti    File index added, bug fixed on UtlTim_FSM callback handle timer management
   *  22/12/22   0.12.0      Francesco Alberti    Replaced UtlTim_isInUse --> UtlTim_isValidHandle.
   *                                              Replaced enum with define UTLTIM_SW_NUM_MAX
   *  22/12/22   0.12.1      Mirko Grillo         Fix Assert Definitions
   *  22/12/22   0.12.2      Mirko Grillo         Fix Assert Definitions
   *  05/01/23   0.12.3      D.F.Granada          Normalize target and compiler.
   *  12/01/23   0.12.4      Mirko Grillo         Fix Assert Definitions
   *  13/01/23   0.12.5      D.F.Granada          Update AFI list.
   *  16/01/23   0.12.6      Mirko Grillo         Update AFI list.
   *  16/01/23   0.12.7      Mirko Grillo         fix bug
   *  20/01/23   0.12.8      Francesco Alberti    Bug Fix on UtlTim assert management.
   *                                              Added define to enable or disable the log according to the selected log level.
   *                                              Added semaphore to UtlLog.
   *                                              Added new assert function.
   *  23/01/23   0.12.9      Francesco Alberti    Added cppcheck macro, fixed cppcheck warnings
   *  24/01/23   0.12.10     S.Venditti           Update AFI list.
   *  25/01/23   0.12.11     Francesco Alberti    Update AFI list. Added Memory Macro functions and CPPCHECK_SUPPRESS on log module
   *  26/01/23   0.12.12     Mirko Grillo         erased assert debugonly
   *  26/01/23   0.12.13     Mirko Grillo         critical sections
   *  30/01/23   0.13.0      Francesco Alberti    Added UtlGen_CrcFlashCalc function.
   *  31/01/23   0.13.1      Francesco Alberti    Added typedef for CRC8/CRC16.
   *  31/01/23   0.13.2      Francesco Alberti    Added automatic critical section, modified UtlTim accordingly
   *  01/02/23   0.13.3      Francesco Alberti    Added semaphore on UtlRbf.
   *  02/02/23   0.13.4      Mirko Grillo         Added calloc "home made"
   *  02/02/23   0.13.5      Francesco Alberti    Added functions to get and set the single module log informations
   *  02/02/23   0.13.6      Francesco Alberti    Modified log informations functions. Added "\n" to log string.
   *  06/02/23   0.13.7      Francesco Alberti    Fixed UtlLog warnings on strcpy / strcat
   *  07/02/23   0.13.8      Francesco Alberti    Fixed misra warnings on UtlMem
   *  14/02/23   0.13.9      Mirko Grillo         Fix Strings on log out
   *  15/02/23   0.13.10     Mattia Zambolin      Updated AFI list.
   *  30/03/23   0.13.11     Mirko Grillo         MISRA rules fixed.
   *  30/03/23   0.14.0      Francesco Alberti    Added CRC32.
   *  31/03/23   0.14.1      Francesco Alberti    Added osSemaphoreDef for semaphore management.
   *  07/04/23   0.14.2      D.F.Granada          Style and refactoring names in RBF.
   *  14/04/23   0.14.3      Mirko Grillo         libcom
   *  27/04/23   0.14.4      Francesco Alberti    changes for common library functionality.
   *  05/05/23   0.14.6      D.F.Granada          New Macros for defining multiple static semaphores
   *  07/06/23   0.14.7      S.Venditti           Some defines added in the wrapper
   *  14/06/23   0.14.8      M.Grillo             Fix dfnp
   *  13/07/23   0.14.9      Francesco Alberti    Added wrapper functions and AFI define
   *  24/07/23   0.15.0      M.Grillo             cmsis v2
   *  25/07/23   0.15.1      E.Benso              Minor modifications and bugfixes
   *  26/07/23   0.15.2      M.Grillo             add other cmsis macro, adapt Ring buffer sub library to cmsis 2
   *  01/08/23   0.15.3      D.F.Granada          Cleaning unnecessary includes
   *  28/08/23   0.16.0      A.Ronzani            Add possibility to compile UtlGen with same target alredy existed
   *  30/08/23   0.17.0      Francesco Alberti    Added UTLTIM_CFG_CRITICAL_SECTION defines for timer management.
   *  01/09/23   1.00.0      D.F.Granada          Modified RING BUFFER change signature. New limitation to single producer or consumer in IRQ.
   *  04/09/23   1.00.1      S.Venditti           Support for trimmer and hybrid manager in UtlGen_afi added
   *  05/09/23   1.00.2      D.F.Granada          New AFI code for APLTST
   *  06/09/23   1.01.0      A.Ronzani            Add build assert
   *  06/09/23   1.01.1      G.Cocco              Add new error code
   *  19/12/23   1.01.2      A.Ronzani            Log: Add check length Tag_name
   *  19/01/24   1.01.3      Francesco Alberti    Added include UtlMem_cfg.h in UtlMem.h. Updated lib to ide 1.14 and cubemx 6.10, Bug fix UtlRbf
   *  19/02/24   1.02.0      D.Fiorimonte         Removed UtlHam and updated AFI
   *  26/02/24   1.2.1       M.Grillo             change random function
   *  04/03/24   1.2.2       D.Fiorimonte         Modified and unified CRC functions
   *  07/03/24   1.2.3       D.F.Granada          Misra checks with automatic tool.
   *  03/05/24   1.2.4       R. Paladin           Added AFI code for trimmer drivers component
   *  25/07/24   1.2.5       D.F.Granada          Cmake added/modify link libraries.
   *  19/09/24   1.3.0       D.F.Granada          Deleted UTLSTM
   *  22/10/24   2.0.0       M. Grillo            modify for right positioning of the macro, add functions on osl
   *
   *******************************************************************************/
   /** @file LibUtl.h
    *  @brief This module manage the general utilities/macros that can fit into this module.
    *  Asserts
    *  Generic error codes
    *
    */
#ifndef UTLGEN_H
#define UTLGEN_H

#ifdef __cplusplus
extern "C" {
#endif

    /******************************************************************************
     * Includes
     *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "UtlGen_cfg.h" /*needed for UTLGEN_ASSERT_ENABLE*/
     /******************************************************************************
      * Preprocessor Constants
      *******************************************************************************/
      /** Major version number (X.x.x) */
#define UTLGEN_VERSION_MAJOR   2
/** Minor version number (x.X.x) */
#define UTLGEN_VERSION_MINOR   0
/** Patch version number (x.x.X) */
#define UTLGEN_VERSION_PATCH   0

/**
 * \brief Macro used for build a string from a integer define.
 * Use UTLGEN_TO_STR, the TO_STR_ haven't the UTLGEN prefix because is only for internal use
 *
 */
#define TO_STR_(x) #x
#define UTLGEN_TO_STR(x) TO_STR_(x)

 /**
  * Macro to convert version number into an integer
  *
  * To be used in comparisons, such as UTLGEN_VERSION >= UTLGEN_VERSION_VAL(1, 0, 0)
  */
#define UTLGEN_VERSION_VAL(major, minor, patch) (((major) << 16) | (minor << 8) | (patch))

  /**
   * Current library version, as an integer
   *
   * To be used in comparisons, such as UTLGEN_VERSION >= UTLGEN_VERSION_VAL(1, 0, 0)
   */
#define UTLGEN_VERSION  UTLGEN_VERSION_VAL( UTLGEN_VERSION_MAJOR, \
                                            UTLGEN_VERSION_MINOR, \
                                            UTLGEN_VERSION_PATCH )

   /**
    * Current Module version, as a string
    */
#define UTLGEN_VERSION_STRING    TO_STR(UTLGEN_VERSION_MAJOR)"."\
                                 TO_STR(UTLGEN_VERSION_MINOR)"."\
                                 TO_STR(UTLGEN_VERSION_PATCH)

    /******************************************************************************
     * Configuration Constants
     *******************************************************************************/
     /**
      * @brief return error code typedef, can be extended defining module specific error codes
      */
    typedef int32_t UtlGen_Err_t;

    /**
     * @brief Common return codes
     */
#define UTLGEN_OK                      (UtlGen_Err_t)(0)      /*!< LibUtl_err_t value indicating success (no error) */
#define UTLGEN_FAIL                    (UtlGen_Err_t)(-1)     /*!< Generic LibUtl_err_t code indicating failure     */

#define UTLGEN_ERR_NO_MEM              (UtlGen_Err_t)(-2)     /*!< Out of memory                      */
#define UTLGEN_ERR_INVALID_ARG         (UtlGen_Err_t)(-3)     /*!< Invalid argument                   */
#define UTLGEN_ERR_INVALID_STATE       (UtlGen_Err_t)(-4)     /*!< Invalid state                      */
#define UTLGEN_ERR_INVALID_SIZE        (UtlGen_Err_t)(-5)     /*!< Invalid size                       */
#define UTLGEN_ERR_NOT_FOUND           (UtlGen_Err_t)(-6)     /*!< Requested resource not found       */
#define UTLGEN_ERR_NOT_SUPPORTED       (UtlGen_Err_t)(-7)     /*!< Operation or feature not supported */
#define UTLGEN_ERR_TIMEOUT             (UtlGen_Err_t)(-8)     /*!< Operation timed out                */
#define UTLGEN_ERR_INVALID_RESPONSE    (UtlGen_Err_t)(-9)     /*!< Received response was invalid      */
#define UTLGEN_ERR_INVALID_CRC         (UtlGen_Err_t)(-10)    /*!< CRC or checksum was invalid        */
#define UTLGEN_ERR_INVALID_VERSION     (UtlGen_Err_t)(-11)    /*!< Version was invalid                */
#define UTLGEN_ERR_INVALID_MAC         (UtlGen_Err_t)(-12)    /*!< MAC address was invalid            */
#define UTLGEN_ERR_BUSY                (UtlGen_Err_t)(-13)    /*!< Peripheral is busy                 */
#define UTLGEN_ERR_NOT_READY           (UtlGen_Err_t)(-14)    /*!< Peripheral/source is not ready     */
#define UTLGEN_ERR_BUS_CONTENTION      (UtlGen_Err_t)(-15)    /*!< Bus contention detected            */


     /******************************************************************************
      * Typedefs and structures
      *******************************************************************************/
      /*CRC-8 typedef*/
    typedef uint8_t    UTLGEN_CRC8;

    /*CRC-16 typedef*/
    typedef uint16_t   UTLGEN_CRC16;

    /*CRC-32 typedef*/
    typedef uint32_t   UTLGEN_CRC32;

    typedef enum
    {
        UTLGEN_CRC_OK = 0,
        UTLGEN_WRONG_CRC = 1,
        UTLGEN_ERROR_ADDRESS_TYPE_CRC = 0xfe,
        UTLGEN_MISSING_CRC = 0xff
    } UtlGen_CrcResult_t;


    /******************************************************************************
     * Macros
     *******************************************************************************/

#define UTLGEN_ASSERT( x )             do{if(false==!!(x)){UTLGEN_CFG_ASSERT_FUNCTION();}}while(0)

//// @todo: NON COMPILA !!!
//// #define UTLGEN_BUILD_ASSERT(EXPR, MSG...) UTLGEN_CFG_BUILD_ASSERT_FUNCTION(EXPR, MSG)

#if defined(UTLGEN_CFG_ASSERT_ENABLE)
#define UTLGEN_ASSERT_DEV(err)         do{if(UTLGEN_OK!=(err)){UTLGEN_CFG_ASSERT_DEV_FUNCTION(err);}}while(0)
#else

#define UTLGEN_ASSERT_DEV(err)
#endif


     /* return the number of elements in an array
      * from Google's Chromium project):
      * It improves on the array[0] or *array version by using 0[array],
      * which is equivalent to array[0] on plain arrays, but will fail to compile
      * if array happens to be a C++ type that overloads operator[]() */
#define UTLGEN_COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

      /**
       * @def UTLGEN_BITSPERWORD
       * @brief Get the number of bits in one word
       *
       */
#define UTLGEN_BITSPERWORD     (UTLGEN_BITSPERBYTE * sizeof(uint16_t))

       /**
        * @def UTLGEN_BITSPERDWORD
        * @brief Get the number of bits in one dword
        *
        */
#define UTLGEN_BITSPERDWORD    (UTLGEN_BITSPERBYTE * sizeof(uint32_t))

        /**
         * \brief SWAP the bytes of a uint32_t
         */
#define UTLGEN_SWAP_32( dWord )    do{*dWord = (((uint32_t)(*dWord)>> 24u ) & 0xffu )                               |  /* move byte 3 to byte 0  */ \
                                               (((uint32_t)((uint32_t)(*dWord) << UTLGEN_BITSPERBYTE)) & 0xff0000u) |  /* move byte 1 to byte 2  */ \
                                               (((uint32_t)((uint32_t)(*dWord) >> UTLGEN_BITSPERBYTE)) & 0xff00u )  |  /* move byte 2 to byte 1  */ \
                                               (((uint32_t)((uint32_t)(*dWord) << 24u            )) & 0xff000000u );   /* byte 0 to byte 3       */ \
                                      }while(0)
         /**
          * \brief SWAP the bytes of a uint16_t
          */
#define UTLGEN_SWAP_16( word )      do{*word=((uint16_t)(*word)>>UTLGEN_BITSPERBYTE)|((uint16_t)((uint16_t)(*word)<<UTLGEN_BITSPERBYTE));}while(0)
          /**
           * @def UTLGEN_BIT
           * @brief
           *
           */
#define UTLGEN_BIT(n)    (1U << (n))

           /**
            * @def UTLGEN_BIT_L
            * @brief
            *
            */
#define UTLGEN_BIT_L(n)    ( 1ULL << (n) )

            /**
             * @def UTLGEN_BITMASK
             * @brief
             *
             */
#define UTLGEN_BITMASK(field_width) ( UTLGEN_BIT(field_width) - 1U )

             /**
              * @def UTLGEN_BITMASK_L
              * @brief
              *
              */
#define UTLGEN_BITMASK_L(field_width) ( UTLGEN_BIT_L(field_width) - 1ULL )

              /**
               * @def UTLGEN_FIELDOFFSET
               * @brief Get the offset of a specific struct field
               *
               */
#define UTLGEN_FIELDOFFSET(type, field)    ((uint32_t)(&(((type) *)0)->(field)))

               /**
                * @def UTLGEN_BITSPERBYTE
                * @brief Bits number in one byte
                *
                */
#define UTLGEN_BITSPERBYTE     8u

                /**
                 * @brief Half bits number in one byte
                 *
                 */
#define UTLGEN_HALFBITSPERBYTE     (UTLGEN_BITSPERBYTE / 2)

                 /**
                  * @def UTLGEN_BITSPERWORD
                  * @brief Get the number of bits in one word
                  *
                  */
#define UTLGEN_BITSPERWORD     (UTLGEN_BITSPERBYTE * sizeof(uint16_t))

                  /**
                   * @def UTLGEN_BITSPERDWORD
                   * @brief Get the number of bits in one dword
                   *
                   */
#define UTLGEN_BITSPERDWORD    (UTLGEN_BITSPERBYTE * sizeof(uint32_t))

                   /**
                    * @def UTLGEN_ODD
                    * @brief Check if the n value is odd
                    *
                    */
#define UTLGEN_ODD(n) ((n) & 1u)

                    /**
                     * @def UTLGEN_LO
                     * @brief Get the lower byte of a word
                     *
                     */
#define UTLGEN_LO(n)  (uint8_t)((n) & UTLGEN_BITMASK(UTLGEN_BITSPERBYTE))

                     /**
                      * @def UTLGEN_HI
                      * @brief Get the upper byte of a word
                      *
                      */
#define UTLGEN_HI(n)  (uint8_t)((n) >> UTLGEN_BITSPERBYTE)

                      /**
                       * @def UTLGEN_MIN
                       * @brief Return the smallest of "a" and "b"
                       *
                       */
#define UTLGEN_MIN(a,b)    (((a) < (b)) ? (a) : (b))

                       /**
                        * @def UTLGEN_MAX
                        * @brief Return the largest of "a" and "b"
                        *
                        */
#define UTLGEN_MAX(a,b)    (((a) > (b)) ? (a) : (b))

                        /**
                         * \brief starting val for the crc8 calc function
                         */
#define UTLGEN_CRC8_STARTING_VAL    0u

                         /**
                          * \brief starting val for the crc16 calc function
                          */
#define UTLGEN_CRC16_STARTING_VAL    (UTLGEN_CRC16)(~ 0)

                          /**
                           * \brief starting val for the crc32 calc function
                           */
#define UTLGEN_CRC32_STARTING_VAL    (UTLGEN_CRC32)(~ 0)

                           /**
                            * \brief To avoid unused parameter compiler warnings
                            */
#define UTLGEN_UNUSED(X) (void)(X)

                            /**
                             * @def USING_RTOS
                             * @brief Used by lib where the code could be use also without RTOS
                             *
                             */

#ifdef UTLGEN_CFG_USE_RTOS
#define UTLGEN_USE_RTOS
#endif

#ifdef UTLGEN_CFG_WEAK
#define UTLGEN_WEAK UTLGEN_CFG_WEAK
#else
#define UTLGEN_WEAK
#endif


#ifdef UTLGEN_CFG_FORCE_INLINE
#define UTLGEN_FORCE_INLINE UTLGEN_CFG_FORCE_INLINE
#else
#define UTLGEN_FORCE_INLINE
#endif

                             /******************************************************************************
                              * Instance Definitions
                              *******************************************************************************/

                              /******************************************************************************
                               * Variables
                               *******************************************************************************/

                               /******************************************************************************
                                * Function Definitions/Prototypes for PUBLIC functions
                                *******************************************************************************/
                                /**
                                 * @brief Function for calculating CRC-8 of one byte (polynomial 0x07) with 0x00 initial value.
                                 *
                                 * @param[in] Data   Byte to calculate the CRC-8.
                                 * @param[in] Crc    Starting CRC value.
                                 *
                                 * @return    Calculated CRC-8.
                                 */
    UTLGEN_CRC8 UtlGen_UpdateCRC8(uint8_t Data, UTLGEN_CRC8 Crc);

    /**
     * @brief Function for calculating CRC-8 of a buffer of bytes (polynomial 0x07) with 0x00 initial value.
     *
     * Feed each consecutive data block into this function, along with the current value of CrcInitValue as
     * returned by the previous call of this function. The first call of this function should pass
     * UTLGEN_CRC8_STARTING_VAL as the initial value of the CRC in CrcInitValue.
     * The watchdog reset must be done by the caller!
     *
     * @param[in] pDataIn        Pointer to the buffer of which calc the CRC-8.
     * @param[in] DataLen        Number of bytes to use.
     * @param[in] CrcInitValue   The previous calculated CRC-8 value or UTLGEN_CRC8_STARTING_VAL if first call.
     *
     * @return    The updated CRC-8 value, based on the input supplied.
     */
    UTLGEN_CRC8 UtlGen_CalcBufferCRC8(const uint8_t* pDataIn, uint32_t DataLen, UTLGEN_CRC8 CrcInitValue);

    /**
     * @brief  Function for calculating CRC-16/CCITT-FALSE of one byte (polynomial 0x1021) with 0xFFFF initial value.
     *
     * @param[in] Data   Byte to calculate the CRC-16.
     * @param[in] Crc    Starting CRC value.
     *
     * @return    Calculated CRC-16.
     */
    UTLGEN_CRC16 UtlGen_UpdateCRC16(uint8_t Data, UTLGEN_CRC16 Crc);

    /**
     * @brief Function for calculating CRC-16/CCITT-FALSE in blocks (polynomial 0x1021) with 0xFFFF initial value.
     *
     * Feed each consecutive data block into this function, along with the current value of CrcInitValue as
     * returned by the previous call of this function. The first call of this function should pass
     * UTLGEN_CRC16_STARTING_VAL as the initial value of the CRC in CrcInitValue.
     * The watchdog reset must be done by the caller!
     *
     * @param[in] pDataIn        The input data block for computation.
     * @param[in] DataLen        The size of the input data block in bytes.
     * @param[in] CrcInitValue   The previous calculated CRC-16 value or UTLGEN_CRC16_STARTING_VAL if first call.
     *
     * @return    The updated CRC-16 value, based on the input supplied.
     */
    UTLGEN_CRC16 UtlGen_CalcBufferCRC16(const uint8_t* pDataIn, uint32_t DataLen, UTLGEN_CRC16 CrcInitValue);

    /**
     * @brief Function for calculating CRC-32/MPEG-2 of one byte (polynomial 0x04C11DB7) with 0xFFFFFFFF initial value.
     *
     * @param[in] Data   Byte to calculate CRC-32.
     * @param[in] Crc    Starting CRC value.
     *
     * @return    Calculated CRC-32.
     */
    UTLGEN_CRC32 UtlGen_UpdateCRC32(uint8_t Data, UTLGEN_CRC32 Crc);

    /**
     * @brief Function for calculating CRC-32/MPEG-2 of a buffer of bytes (polynomial 0x04C11DB7) with 0xFFFFFFFF initial value.
     *
     * Feed each consecutive data block into this function, along with the current value of CrcInitValue as
     * returned by the previous call of this function. The first call of this function should pass
     * UTLGEN_CRC32_STARTING_VAL as the initial value of the CRC in CrcInitValue.
     * The watchdog reset must be done by the caller!
     *
     * @param[in] pDataIn        The input data block for computation.
     * @param[in] DataLen        The size of the input data block in bytes.
     * @param[in] CrcInitValue   The previous calculated CRC-32 value or UTLGEN_CRC32_STARTING_VAL if first call.
     *
     * @return    The updated CRC-32 value, based on the input supplied.
     */
    UTLGEN_CRC32 UtlGen_CalcBufferCRC32(const uint8_t* pDataIn, uint32_t DataLen, UTLGEN_CRC32 CrcInitValue);

    /**
     * \brief If the counter is not expired, this function decreases it.
     * After that if the counter is expired, the function will reload it
     *
     * \param[in] pCounter  Counter Pointer
     * \param[in] DecValue  Counter Decrease Value
     * \param[in] Val       New Reloaded Value
     * \return  true in falling of counter towards zero and reload the counter, if counter is already 0 return false
     */
    bool UtlGen_IsDecExpiredReload(uint32_t* pCounter, uint32_t DecValue, uint32_t Val);

    /**
     * \brief If the counter is not expired, this function decreases it.
     *
     * \param[in] pCounter  Counter Pointer
     * \param[in] DecValue  Counter Decrease Value
     * \return true in falling of counter towards zero, if counter is already 0 return false
     */
    bool UtlGen_IsDecExpiredNow(uint32_t* pCounter, uint32_t DecValue);

    /**
     * \brief This function decreases the counter and
     * checks if it is expired
     *
     * \param[in] pCounter  Counter Pointer
     * \param[in] DecValue  Counter Decrease Value
     * \return true in falling of counter towards zero, if counter is already 0 return TRUE
     */
    bool UtlGen_IsDecExpired(uint32_t* pCounter, uint32_t DecValue);


    /**
     * @brief Reload the counter
     * @param pCounter
     * @param Val
     * @return
     */
    UtlGen_Err_t UtlGen_DecReload(uint32_t* pCounter, uint32_t Val);

    /**
     * @brief Reset to zero the counter (no event more)
     * @param pCounter
     * @return
     */
    UtlGen_Err_t UtlGen_DecReset(uint32_t* pCounter);


    /**
     * \brief This function generates a random number. Generation is either sw- or hw-based (where possibile, the latter must be defined in the config)
     * sw-based: Width varies according to RAND_MAX value and implementation (min 15 bits)
     * hw-based: 32-bit width
     * @return RAND_MAX-wide  random number
     */
    uint32_t UtlGen_GenRandomNumber(void);

    /**
     * \brief This function extract a conform address from a stream of bytes
     *
     *
     * \param[in] pSourceVal  source byte stream
     * \param[in] NVal        number of bytes
     * \return UBaseType_t var with the N bytes aligned
     */
    /// @todo: NON COMPILA!!!
    /// UTLGEN_CFG_ADDRESS_TYPE UtlGen_GetNBytes(const uint8_t* pSourceVal, uint8_t NVal);

    /**
     * \brief This function calculates and validates CRC-8 for FLASH.
     *
     *
     * \return UtlGen_CrcResult_t CrcResult
     */
    UtlGen_CrcResult_t UtlGen_CrcFlashCalc(void);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* UTLGEN_H */

/*** End of File **************************************************************/
