#ifndef TEST_DRV_CAN_H
#define TEST_DRV_CAN_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U
} HAL_StatusTypeDef;

typedef struct {
    uintptr_t instance;
} CAN_HandleTypeDef;

extern CAN_HandleTypeDef hcan2;

HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan,
                               uint32_t std_id,
                               uint8_t *data);

#endif
