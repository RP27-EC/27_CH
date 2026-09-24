#include "kt_yaw_motor.h"

KT_motor_t kt_motor[1] = {
    {
        .KT_motor_info = {
            .id = {
                .tx_id = KT_YAW_TX_ID,
                .rx_id = KT_YAW_RX_ID,
                .drive_type = M_CAN1,
                .motor_type = KT9025,
            },
        },
        .init = KT_motor_class_init,
    },
};

void KT_Yaw_Init(void)
{
    kt_motor[0].init(&kt_motor[0]);
}

void KT_Yaw_Heartbeat(void)
{
    kt_motor[0].heartbeat(&kt_motor[0]);
}

void KT_Yaw_SendIq(int16_t iq)
{
    kt_motor[0].W_iqControl(&kt_motor[0], iq);
    kt_motor[0].tx_W_cmd(&kt_motor[0], TORQUE_CLOSE_LOOP_ID);
}
