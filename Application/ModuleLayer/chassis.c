#include "chassis.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 声明全局底盘对象 (定义在 control_task.c 中) */
extern Chassis_t hero_chassis;

static float Float_Clamp(float val, float min, float max) {
    if (!isfinite(val)) return 0.0f;
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

static float PID_Calculate(Chassis_PID_t *pid, float target, float feedback) {
    float error = target - feedback;

    float p_out = pid->kp * error;

    pid->integral += pid->ki * error;
    pid->integral = Float_Clamp(pid->integral, -pid->max_iout, pid->max_iout);
    float i_out = pid->integral;

    float d_out = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;

    float total_out = p_out + i_out + d_out;
    return Float_Clamp(total_out, -pid->max_out, pid->max_out);
}

void Chassis_Init(Chassis_t *chassis) {
    memset(chassis, 0, sizeof(Chassis_t));

    for (int i = 0; i < 4; i++) {
        chassis->wheel[i].pid.kp = CHASSIS_PID_KP;
        chassis->wheel[i].pid.ki = CHASSIS_PID_KI;
        chassis->wheel[i].pid.kd = CHASSIS_PID_KD;
        chassis->wheel[i].pid.max_out = CHASSIS_MAX_CURRENT;
        chassis->wheel[i].pid.max_iout = CHASSIS_PID_MAX_IOUT;
    }
    chassis->enabled = false;
}

/**
 * @brief 麦轮逆解算 + 4轮速度 PID + 发送 CAN 报文 (无死区纯线性映射)
 */
void Chassis_Control_Loop(Chassis_t *chassis, int16_t ch_vx, int16_t ch_vy, int16_t ch_wz, bool failsafe, CAN_HandleTypeDef *hcan) {
    /* 1. 安全关控检查 */
    if (failsafe) {
        chassis->enabled = false;
        for (int i = 0; i < 4; i++) {
            chassis->wheel[i].give_current = 0;
            chassis->wheel[i].pid.integral = 0.0f;
            chassis->wheel[i].pid.prev_error = 0.0f;
        }
    } else {
        chassis->enabled = true;

        /* 2. 直接归一化为 [-1.0, +1.0] 的浮点速度比例 (无死区处理) */
        chassis->vx = (float)ch_vx / CHASSIS_RC_MAX_VALUE;
        chassis->vy = (float)ch_vy / CHASSIS_RC_MAX_VALUE;
        chassis->wz = (float)ch_wz / CHASSIS_RC_MAX_VALUE;

        /* 3. 麦轮逆解算 (标准 O 型，ROS/RM 权威右手坐标系) */
        float raw_rpm[4];
        raw_rpm[0] = (+chassis->vx - chassis->vy - chassis->wz) * CHASSIS_MAX_WHEEL_RPM; // 0: FL
        raw_rpm[1] = (+chassis->vx + chassis->vy + chassis->wz) * CHASSIS_MAX_WHEEL_RPM; // 1: FR
        raw_rpm[2] = (+chassis->vx + chassis->vy - chassis->wz) * CHASSIS_MAX_WHEEL_RPM; // 2: BL
        raw_rpm[3] = (+chassis->vx - chassis->vy + chassis->wz) * CHASSIS_MAX_WHEEL_RPM; // 3: BR

        /* 4. 等比例归一化限幅 (防超速偏航) */
        float max_rpm = 0.0f;
        for (int i = 0; i < 4; i++) {
            float abs_rpm = fabsf(raw_rpm[i]);
            if (abs_rpm > max_rpm) max_rpm = abs_rpm;
        }
        if (max_rpm > CHASSIS_MAX_WHEEL_RPM) {
            float rate = CHASSIS_MAX_WHEEL_RPM / max_rpm;
            for (int i = 0; i < 4; i++) raw_rpm[i] *= rate;
        }

        /* 5. 4轮独立速度 PID 闭环 (右侧电机 1号 FR, 3号 BR 硬件镜像取反) */
        for (int i = 0; i < 4; i++) {
            chassis->wheel[i].target_rpm = raw_rpm[i];
            float target = (i == 1 || i == 3) ? -chassis->wheel[i].target_rpm : chassis->wheel[i].target_rpm;
            float feedback = (float)chassis->wheel[i].feedback_rpm;
            chassis->wheel[i].give_current = (int16_t)PID_Calculate(&chassis->wheel[i].pid, target, feedback);
        }
    }

    /* 6. 组装 0x200 标准 CAN 帧并发送 */
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t mailbox;

    header.StdId = 0x200;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    data[0] = (uint8_t)(chassis->wheel[0].give_current >> 8);
    data[1] = (uint8_t)(chassis->wheel[0].give_current & 0xFF);
    data[2] = (uint8_t)(chassis->wheel[1].give_current >> 8);
    data[3] = (uint8_t)(chassis->wheel[1].give_current & 0xFF);
    data[4] = (uint8_t)(chassis->wheel[2].give_current >> 8);
    data[5] = (uint8_t)(chassis->wheel[2].give_current & 0xFF);
    data[6] = (uint8_t)(chassis->wheel[3].give_current >> 8);
    data[7] = (uint8_t)(chassis->wheel[3].give_current & 0xFF);

    (void)HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

/**
 * @brief 在 CAN 接收中断中调用：解析 0x201 ~ 0x204 电调反馈
 */
void Chassis_Parse_Feedback(Chassis_t *chassis, uint32_t std_id, const uint8_t data[8]) {
    if (std_id >= 0x201 && std_id <= 0x204) {
        uint8_t i = std_id - 0x201;
        chassis->wheel[i].feedback_ecd = (uint16_t)((data[0] << 8) | data[1]);
        chassis->wheel[i].feedback_rpm = (int16_t)((data[2] << 8) | data[3]);
        chassis->wheel[i].feedback_current = (int16_t)((data[4] << 8) | data[5]);
    }
}

/**
 * @brief 覆盖 drv_can.c 中的 __WEAK 弱函数，自动无缝接管 CAN1 接收数据流！
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf) {
    Chassis_Parse_Feedback(&hero_chassis, rxId, rxBuf);
}
