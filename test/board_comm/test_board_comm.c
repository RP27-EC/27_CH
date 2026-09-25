#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board_comm.h"

CAN_HandleTypeDef hcan2 = {2U};

static CAN_HandleTypeDef *captured_hcan;
static uint32_t captured_id;
static uint8_t captured_data[8];
static HAL_StatusTypeDef send_result = HAL_OK;

HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan,
                               uint32_t std_id,
                               uint8_t *data)
{
    captured_hcan = hcan;
    captured_id = std_id;
    memcpy(captured_data, data, sizeof(captured_data));
    return send_result;
}

int main(void)
{
    float yaw;
    float rate;

    Board_Comm_Init();
    assert(Board_Comm_Send_IMU(1.25f, -2.5f) == HAL_OK);
    assert(captured_hcan == &hcan2);
    assert(captured_id == MASTER_RX_YAW_IMU_CAN_ID);

    memcpy(&yaw, &captured_data[0], sizeof(yaw));
    memcpy(&rate, &captured_data[4], sizeof(rate));
    assert(yaw == 1.25f);
    assert(rate == -2.5f);

    send_result = HAL_ERROR;
    assert(Board_Comm_Send_IMU(0.0f, 0.0f) == HAL_ERROR);

    puts("board_comm tests passed");
    return 0;
}
