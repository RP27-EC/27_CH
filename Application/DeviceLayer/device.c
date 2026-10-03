/**
 * @file    device.c
 * @brief   设备层初始化总入口文件
 *
 * 本文件是设备层的顶层初始化入口，负责调用所有子设备的初始化函数。
 * DEVICE_Init() 在系统启动时被 main() 调用，按照顺序依次初始化：
 *   1. IMU 传感器（陀螺仪 + 加速度计）
 *   2. 遥控器传感器
 *   3. 各类电机驱动（RM电机、HT电机、DM电机、KT电机）
 *
 * 所有设备的初始化均采用面向对象风格（函数指针/结构体封装），
 * 每个子设备模块对外暴露 init、update、heart_beat 等统一接口。
 *
 * @note   在 main.c 中的 main() 函数启动时调用 DEVICE_Init() 完成所有硬件初始化
 * @version V1.0
 * @date    2026-06-11
 */

/* Includes ------------------------------------------------------------------*/
#include "device.h"  /* 包含所有设备层头文件的统一入口头文件 */

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  设备层总初始化函数
 *
 * 按照以下顺序完成所有外设的初始化：
 *   1. imu_sensor.init()    - 初始化 IMU 陀螺仪/加速度计传感器（BMI088）
 *   2. rc_sensor.init()     - 初始化遥控器传感器（DBUS 协议接收）
 *   3. rm_motor_list_init() - 初始化 RoboMaster 系列电机（6020 等，CAN 总线）
 *   4. kt_motor_list_init() - 初始化 KT 系列电机（KT9015 等，CAN 总线）
 *   5. ht_motor_list_init() - 初始化 HT 系列电机（轮毂电机，CAN 总线）
 *   6. dm_motor_list_init() - 初始化 DM 系列电机（达妙电机，CAN 总线）
 *
 * @note   此函数在 main() 中调用，调用前需确保 CAN、UART、SPI 等外设时钟已开启
 * @return 无返回值
 */
void DEVICE_Init(void)
{

	imu_sensor.init(&imu_sensor);

	rc_sensor.init(&rc_sensor);

	rm_motor_list_init();

  kt_motor_list_init();

	ht_motor_list_init();

	dm_motor_list_init();

}
