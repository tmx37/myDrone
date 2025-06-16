#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// TODO: add PWM timer handler

uint32_t DrvPWM_WriteValue(uint32_t PWMValue);

#ifdef __cplusplus
}
#endif