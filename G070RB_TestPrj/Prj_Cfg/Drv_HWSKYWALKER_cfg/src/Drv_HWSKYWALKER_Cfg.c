/*
 * Drv_HWSKYWALKER_Cfg.c
 *
 *  Created on: 20/11/2026
 *      Author: tmx37
 */

#include "Drv_HWSKYWALKER_Cfg.h"
#include "Drv_HWSKYWALKER.h"

static Drv_HWSKYWALKER_Config_t mConfigData;

GenUtl_Err_t UART_Trasmit_Wrapper(const uint8_t* pData, uint16_t Size, uint32_t Timeout);
GenUtl_Err_t PWM_Set_Wrapper(uint32_t Value);

GenUtl_Err_t Drv_HWSKYWALKER_Cfg_Init()
{
    GenUtl_Err_t ErrCode;
    mConfigData.pfUartTrasmitCB = UART_Trasmit_Wrapper;
    mConfigData.pfSetPwmCB = PWM_Set_Wrapper;
    mConfigData.pfDelayCB = HAL_Delay;
    ErrCode = Drv_HWSKYWALKER_Init(&mConfigData);
    return ErrCode;
}

/**
 * Add here mock functions
 */
GenUtl_Err_t UART_Trasmit_Wrapper(const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    /** TODO: forward to LM_Comms */
    (void)pData; 
    (void)Size; 
    (void)Timeout;
    return OUT_OK;
}

GenUtl_Err_t PWM_Set_Wrapper(uint32_t Value)
{
    /** NB: This handles only one timer at the time. Consider to add a "Timer index" or multi-instance*/
    TIM1->CCR1 = Value;
    return OUT_OK;
}

