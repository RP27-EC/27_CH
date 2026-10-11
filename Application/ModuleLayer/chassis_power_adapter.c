#include "chassis_power.h"
#include "chassis_motor.h"
#include "judge.h"
#include "cap.h"

void Chassis_Power_Apply(void) {
    ChassisPowerInput_t input = {0};
    float current[4], rpm[4];
    /* 接收中断会同时改数据、时间和序号，短临界区保证拿到同一帧快照。 */
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    uint32_t now_ms = HAL_GetTick();
    input.limit_received = My_Judge.status->limit_received;
    input.limit_seq = My_Judge.status->limit_rx_count;
    input.limit_ms = My_Judge.status->limit_update_ms;
    input.limit_w = (float)My_Judge.org_info->game_robot_status.chassis_power_limit;
    input.buffer_received = My_Judge.status->buffer_received;
    input.buffer_seq = My_Judge.status->buffer_rx_count;
    input.buffer_ms = My_Judge.status->buffer_update_ms;
    input.buffer_j = (float)My_Judge.org_info->power_heat_data.chassis_power_buffer;
    input.cap_received = cap.status->power_received;
    input.cap_seq = cap.status->power_rx_count;
    input.cap_ms = cap.status->power_update_ms;
    input.cap_w = (float)cap.info->chassis_power;
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) {
        current[i] = Chassis_Motor[i].tx_info->torque;
        rpm[i] = (float)Chassis_Motor[i].rx_info->encoder_speed;
    }
    __set_PRIMASK(irq_mask);

    Chassis_Power_Update(&input, now_ms);
    Chassis_Power_Limit(current, rpm);
    for (unsigned i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
        Chassis_Motor[i].tx_info->torque = current[i];
}
