#ifndef KT_YAW_MOTOR_H
#define KT_YAW_MOTOR_H

#include "KT_motor.h"

#define KT_YAW_RX_ID 0x142U
#define KT_YAW_TX_ID 0x142U

extern KT_motor_t kt_motor[1];

void KT_Yaw_Init(void);
void KT_Yaw_Heartbeat(void);
void KT_Yaw_SendIq(int16_t iq);

#endif
