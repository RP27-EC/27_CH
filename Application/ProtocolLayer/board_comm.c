#include "board_comm.h"

#include <string.h>

extern CAN_HandleTypeDef hcan2;

void Board_Comm_Init(void)
{
}

HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad,
                                      float yaw_rate_rad_s)
{
    uint8_t tx_buf[8];

    memcpy(&tx_buf[0], &yaw_angle_rad, sizeof(float));
    memcpy(&tx_buf[4], &yaw_rate_rad_s, sizeof(float));

    return CAN_SendData(&hcan2, MASTER_RX_YAW_IMU_CAN_ID, tx_buf);
}
