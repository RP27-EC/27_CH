/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   总控任务 (Control Task) —— 底盘全向移动 + IMU 姿态解算更新
  ******************************************************************************
  */
#include "control_task.h"
#include "can.h"

/* 外部 CAN 句柄声明 */
extern CAN_HandleTypeDef hcan1;

/* 全局底盘控制对象定义 */
Chassis_t hero_chassis;

/**
 * @brief 总控任务主体
 */
void StartControlTask(void const * argument)
{
    /* 1. 任务启动前的安全缓冲等待，并初始化底盘对象与 PID 参数 */
    osDelay(100);
    Chassis_Init(&hero_chassis);

    for(;;) 
    {
        /* 2. 【IMU 姿态传感器更新】(保留战队模板原生代码) */
        if ((imu_sensor.work_state.err_code == IMU_NONE_ERR) || \
            (imu_sensor.work_state.err_code == IMU_DATA_CALI))
        {
            imu_sensor.update(&imu_sensor);
        }

        /* 3. 【关控与安全保护判定】：
           - 战队原生宏 RC_OFFLINE 为真 (遥控器掉线/断电) ➔ 触发保护
           - 右拨杆未置于最下方解锁位 (s2.value != RC_SW_DOWN) ➔ 触发保护
         */
        bool is_failsafe = RC_OFFLINE || (rc_sensor_info.s2.value != RC_SW_DOWN);

        /* 4. 【底盘机械模式控制流水线】：
           - 左摇杆前后 (ch3) ➔ vx 前后平移
           - 左摇杆左右 (ch2) ➔ vy 左右横移
           - 右摇杆左右 (ch0) ➔ wz 原地自旋 (向左推为正，取负号对齐逆时针)
           - 内部自动执行：逆解算 ➔ 归一化限幅 ➔ 4轮速度PID ➔ 打包 0x200 发送至 CAN1
         */
        Chassis_Control_Loop(&hero_chassis,
                             rc_sensor_info.ch3,
                             rc_sensor_info.ch2,
                             -rc_sensor_info.ch0,
                             is_failsafe,
                             &hcan1);

        /* 5. 任务控制周期延时 (2ms 稳定节拍) */
        osDelay(2);
    }
}
