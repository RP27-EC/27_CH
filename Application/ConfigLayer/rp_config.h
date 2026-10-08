/**
 * @file        rp_config.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Configuration.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(7-November-2021)
 *                  1.优化设备类信息与结构体的变量定义，增加volatile/const关键字
 *                  //2.将rp_config.h分成driver_config.h, device_config.h, user_config.h三个头文件    
 */
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "string.h"
// 驱动层配置
#include "rp_driver_config.h"
// 设备层配置
#include "rp_device_config.h"
// 用户层配置
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/
/*有无超电*/
#define CAP_ENABLE		1
/*有无裁判系统*/
#define JUDGE_ENABLE 	1

/*选择IMU解算算法为Mahony*/
#define IMU_USE_MAHONY  0
/*选择IMU解算算法为EKF*/
#define IMU_USE_EKF 	1

/* ========================================================================== */
/*                        底盘功率预算 / 超级电容                              */
/*   已停用：不做功率限制、不读裁判系统。底盘就是 PID + 麦轮直出。            */
/*   power_model 模块已删除。恢复功率限制时再启用下方常量并重建该模块。       */
/* ========================================================================== */

/* 无裁判系统时的训练基准功率上限（W） —— 已停用 */
// #define CHASSIS_POWER_BUDGET_W        (80.0f)
/* 裁判功率限制的裕量系数 —— 已停用 */
// #define CHASSIS_POWER_BUDGET_MARGIN   (1.25f)
/* 电容放宽后的功率封顶（W） —— 已停用 */
// #define CHASSIS_POWER_BUDGET_MAX_W    (150.0f)

/* 超电板失联判定：300 × 1ms = 300ms 收不到 0x211 就判离线（monitor_task 周期是 1ms）。
   超电当前为只读调试：heartbeat 只维护 cap.status，控制路径不消费 */
#define CAP_OFFLINE_CNT_MAX           (300U)
/* 超电控制帧 0x222 发送周期 —— 已停用：TX 路径整体停用（只读调试） */
// #define CAP_TX_PERIOD_TICKS           (10U)
/* 无裁判系统时发给超电板的名义底盘缓冲能量 —— 已停用：不再下发控制帧 */
// #define CAP_TX_BUFFER_NOMINAL         (60U)

/* 电容可放宽功率的电压阈值 —— 已停用：不做电容放宽 */
// #define CAP_RELAX_V_TH                (13.0f)
/* 电压每升高 1V 额外放宽多少瓦 —— 已停用 */
// #define CAP_RELAX_K                   (13.0f)
/* 超过电压阈值后的基础放宽量 —— 已停用 */
// #define CAP_RELAX_BASE_W              (10.0f)
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(uint32_t addr);	//设置堆栈地址 

#endif
