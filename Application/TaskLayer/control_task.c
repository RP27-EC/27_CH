/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   IMU 姿态解算与板间发送任务
  ******************************************************************************
  */
#include "control_task.h"
#include "board_comm.h"

#define DEG_TO_RAD (0.01745329251994329577f)

/**
 * @brief 总控任务主体
 */
void StartControlTask(void const * argument)
{
    /* 任务启动前等待设备初始化完成。 */
    osDelay(100);
    Board_Comm_Init();

    for(;;) 
    {
        /* 2. 【IMU 姿态传感器更新】(保留战队模板原生代码) */
        if ((imu_sensor.work_state.err_code == IMU_NONE_ERR) || \
            (imu_sensor.work_state.err_code == IMU_DATA_CALI))
        {
            imu_sensor.update(&imu_sensor);
            Board_Comm_Send_IMU(imu_sensor.info->base_info.yaw * DEG_TO_RAD,
                                imu_sensor.info->base_info.rate_yaw * DEG_TO_RAD);
        }

        /* 任务控制周期延时 (2ms 稳定节拍)。 */
        osDelay(2);
    }
}
