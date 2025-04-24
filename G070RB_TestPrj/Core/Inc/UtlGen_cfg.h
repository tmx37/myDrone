/****************************************************************************
 * Title                 :   Configuration file of the utilities library
 * Filename              :   UtlGen_cfg.h
 * Author                :   Roberto Monti
 * Origin Date           :   27/05/22
 * Version               :   0.5.1
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
 *  Date       Version   Author              Description
 *  01/01/22   0.1.0     Roberto Monti       Interface Created.
 *  27/05/22   0.5.1     Francesco Alberti   Removed attribute INLINE and PUT in Ram.
 *                                           Moved attribute weak for random number in the CFG.
 *                                           Added UID_BASE define for ARMUUID Address
 *
 *******************************************************************************/
/** @file UTLGEN_CFG.h
 *  @brief This file is used to configure the utilities library
 *
 */
#ifndef UTLGEN_CFG_H
#define UTLGEN_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Includes
 *******************************************************************************/
#include "stdint.h"
#include <assert.h>

/******************************************************************************
 * Preprocessor Constants
 *******************************************************************************/
/* UTLGEN_CFG_RTOS valid values*/
#define UTLGEN_CFG_NO_OS    1
#define UTLGEN_CFG_CMSIS_OS 0
/** configure the operating system interface to be used form the compatible list above */
// #define UTLGEN_CFG_USE_RTOS //TODO to be fixed

/* ARM UUID to be used as a seed for RNG. WARNING: this address is ARM model-dependent */
// #define UTLGEN_CFG_RANDOM_SEED  UID_BASE
#define UTLGEN_CFG_WEAK         __attribute__((weak))
#define UTLGEN_CFG_FORCE_INLINE __attribute__((always_inline))
/******************************************************************************
 * Configuration Constants
 *******************************************************************************/
/** Enable or disable the ASSERT */
// #define UTLGEN_CFG_ASSERT_ENABLE /* comment if not enabled */
#define UTLGEN_CFG_ADDRESS_TYPE uint32_t

/* Not used for this reason set 0 */
#define UTLGEN_CFG_START_ADDRESS_OF_CRC_SEGMENT 0
#define UTLGEN_CFG_END_ADDRESS_OF_CRC_SEGMENT   0

/******************************************************************************
 * Macros
 *******************************************************************************/
#define UTLGEN_CFG_BUILD_ASSERT_FUNCTION(EXPR, MSG...) _Static_assert(EXPR, "" MSG)
/******************************************************************************
 * Typedefs and structures
 *******************************************************************************/

/******************************************************************************
 * Function Definitions/Prototypes for PUBLIC functions
 *******************************************************************************/

/******************************************************************************
 * Macros
 *******************************************************************************/
#define UTLGEN_CFG_ASSERT_FUNCTION()
#define UTLGEN_CFG_ASSERT_DEV_FUNCTION(err) (void)0
#define UTLGEN_ASSERT_DEBUGONLY(x)          // do{if(false==!!(x)){UTLGEN_CFG_ASSERT_FUNCTION();}}while(0)

#define UTLGEN_CFG_SW_RANDOM 0

/******************************************************************************
 * Instance Definitions
 *******************************************************************************/

/******************************************************************************
 * Variables
 *******************************************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* UTLGEN_CFG_H */

/*** End of File **************************************************************/
