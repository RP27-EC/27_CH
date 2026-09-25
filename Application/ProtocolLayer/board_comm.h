#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "drv_can.h"

#define MASTER_RX_YAW_IMU_CAN_ID (0x021U)

void Board_Comm_Init(void);
HAL_StatusTypeDef Board_Comm_Send_IMU(float yaw_angle_rad,
                                      float yaw_rate_rad_s);

#endif
