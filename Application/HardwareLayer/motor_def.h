#ifndef __MOTOR_DEF_H
#define __MOTOR_DEF_H

#include "PID.h"

typedef struct {
    pid_ctrl_t speed;
    pid_ctrl_t angle;
} motor_pid_t;

typedef struct {
    motor_pid_t speed_pid;
    motor_pid_t mec_pid;
    motor_pid_t gyro_pid;
    motor_pid_t position_pid;
    motor_pid_t angle_pid;
    motor_pid_t user_define_pid;
} motor_pid_all_t;

typedef enum { M_OFFLINE = 0, M_ONLINE, M_TYPE_ERR, M_ID_ERR, M_INIT_ERR, M_DATA_ERR } motor_state_e;
typedef enum { M_PROTECT_ON = 0, M_PROTECT_OFF } motor_protect_e;
typedef enum { M_DEINIT = 0, M_INIT } motor_init_e;
typedef enum { M_CAN1, M_CAN2, M_PWM, M_USART1, M_USART2, M_USART3, M_USART4, M_USART5 } motor_drive_e;
typedef enum { GM6020 = 1, RM3508, RM2006, KT9015 = 4, KT9025 } motor_type_e;

#endif
