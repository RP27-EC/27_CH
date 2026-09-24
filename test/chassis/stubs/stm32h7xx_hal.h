#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>

#ifdef errno
#undef errno
#endif

typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier; } FDCAN_RxHeaderTypeDef;
typedef int HAL_StatusTypeDef;

#endif
