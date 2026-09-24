#include "can_protocol.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "cap_protocol.h"
#include "judge.h"
#include "cap.h"
#include "kt_yaw_motor.h"

void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId) {
        case KT_YAW_RX_ID:
            kt_motor[0].get_info(&kt_motor[0], rxBuf);
            break;
        case 0x201:
            Chassis_Motor[0].rx(&Chassis_Motor[0], rxBuf);
            break;
        case 0x202:
            Chassis_Motor[1].rx(&Chassis_Motor[1], rxBuf);
            break;
        case 0x203:
            Chassis_Motor[2].rx(&Chassis_Motor[2], rxBuf);
            break;
        case 0x204:
            Chassis_Motor[3].rx(&Chassis_Motor[3], rxBuf);
            break;
        case 0x012:
            Yaw_Motor.rx(&Yaw_Motor, rxBuf);
            break;
        default:
            break;
    }
}

void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId) {
        case 0x001:
            Yaw_Motor.rx(&Yaw_Motor, rxBuf);
            break;
        default:
            break;
    }
}

void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    (void)rxId;
    (void)rxBuf;
}
