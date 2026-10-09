/**
 * @file    control_task.c
 * @brief   IMU姿态解算与板间发送任务
*/
/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "board_comm.h"
#include "gimbal.h"
#include "shoot.h"          /* 摩擦轮控制模块 */

/* Private macro -------------------------------------------------------------*/
/**
 * @brief  角度单位转换宏：度转弧度
 *         1度 = π/180 ≈ 0.01745329251994329577 弧度
 */
#define DEG_TO_RAD (0.01745329251994329577f)

/* Private functions ---------------------------------------------------------*/


void StartControlTask(void const * argument)
{
    /* 任务启动前等待100ms，确保设备初始化完成 */
    osDelay(100);
    Gimbal_Init();
    Shoot_Init();
    for(;;)
    {
        Board_Comm_Check_Timeout();


        /* 校准期间必须继续采样，但不能允许 Pitch 电机工作。 */
        bool imu_can_update = ((imu_sensor.work_state.err_code == IMU_NONE_ERR) ||
                          (imu_sensor.work_state.err_code == IMU_DATA_CALI));
        if (imu_can_update)
        {
            imu_sensor.update(&imu_sensor);  /* 更新IMU原始数据 */
        }

        /* 使用本次更新后的状态，校准完成且无错误才发布有效姿态。 */
        bool imu_valid = (imu_sensor.work_state.err_code == IMU_NONE_ERR) &&
                         (imu_sensor.work_state.cali_end != 0U);
        if (imu_valid)
        {
            /* 通过CAN总线发送姿态数据给下层板，供下层板进行云台控制 */
            /* 坐标约定：逆时针为正。当前 IMU 的 yaw 角已按该约定输出，
             * 但 rate_yaw 的原始正方向相反，因此发送前取反，使角度和角速度一致。 */
            Board_Comm_Send_IMU(imu_sensor.info->base_info.yaw * DEG_TO_RAD,
                                -imu_sensor.info->base_info.rate_yaw * DEG_TO_RAD);
            /* 与上板闭环一致：Pitch 角度和角速度均以抬头为正。 */
            Board_Comm_Send_Pitch_IMU(imu_sensor.info->base_info.pitch * DEG_TO_RAD,
                                      imu_sensor.info->base_info.rate_pitch * DEG_TO_RAD);
        }

        if (imu_valid) {
            Gimbal_Work();
        } else {
            Gimbal_Safe_Stop();
        }

        /* 摩擦轮控制：消费 0xD1 的 is_fric_on，并写 0x023 状态帧 flags */
        Shoot_Work();

        /*
         * 通过 0x023 发摩擦轮就绪状态给下层板，供其做摩擦轮安全锁。
         * 无条件每个周期发送，下层板才不会因超时退回延时门禁。
         */
        Board_Comm_Send_State(gimbal.pitch.feedback_pitch, imu_valid);

        /* 任务控制周期延时2ms，确保500Hz的稳定控制频率 */
        osDelay(2);
    }
}
