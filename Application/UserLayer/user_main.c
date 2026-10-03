/**
  * @file        user_main.c
  * @author      SSDCFXB
  * @Version     V1.0
  * @date        18-November-2022
  * @brief       用户主程序入口 —— 非RTOS架构的1ms控制循环
  * @details     本文件实现了基于定时器中断（TIM4，1ms周期）的裸机控制架构。
  *              与FreeRTOS任务架构不同，该文件适用于不需要RTOS调度的简单系统。
  *
  *              主要功能：
  *              1. 用户设备初始化（电机、定时器、发射机构）
  *              2. 1ms周期主循环：电机心跳检测、遥控器状态检查、发射机构控制
  *              3. 定时器中断回调：驱动1ms主循环
  *
  *              系统架构：
  *              TIM4定时器(1ms) -> 中断回调 -> user_main() -> 电机心跳 + 发射控制
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"

#include "user_main.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void user_main(void);
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  用户设备初始化函数
  * @details 在系统启动后调用一次，完成以下初始化操作：
  *          1. 所有电机初始化（包括摩擦轮、拨弹轮等）
  *          2. 初始化TIM4定时器（1ms周期）
  *          3. 启动TIM4定时器中断（使能定时器中断回调）
  *          4. 发射机构初始化（摩擦轮校准等）
  *
  * @note   该函数在main()中调用，仅执行一次
  * @retval None
  */
void USER_Init(void)
{
    motor_all_init();                      /* 初始化所有电机（CAN通信、PID参数等） */
    HAL_TIM_Base_Init(&htim4);            /* 初始化TIM4基本定时器（1ms周期） */
    HAL_TIM_Base_Start_IT(&htim4);        /* 启动TIM4中断，使能定时器中断回调 */
    launcher.init();                       /* 发射机构初始化（摩擦轮校准等） */
}


/**
  * @brief  用户应用层主函数（1ms执行一次）
  * @details 该函数由TIM4中断回调触发，每1ms执行一次，完成以下操作：
  *
  *          1. 摩擦轮电机心跳检测：检查右摩擦轮、左摩擦轮是否在线
  *          2. 拨弹轮电机心跳检测：检查拨弹电机是否在线
  *          3. 遥控器心跳检测：检查DBUS通信是否正常
  *          4. 发射机构控制：
  *             - 遥控器在线时：执行正常发射控制逻辑
  *             - 遥控器离线时：执行自我保护（停止发射）
  *
  * @note   该函数在中断上下文中执行，应尽量简短高效
  * @retval None
  */
void user_main(void)
{
    /* ====== 电机心跳检测 ====== */
    /* 心跳检测用于判断电机是否失联（CAN通信中断） */
    motor[FRIC_R].heartbeat(&motor[FRIC_R]);   /* 右摩擦轮电机心跳 */
    motor[FRIC_L].heartbeat(&motor[FRIC_L]);   /* 左摩擦轮电机心跳 */
    motor[DIAL].heartbeat(&motor[DIAL]);       /* 拨弹轮电机心跳 */

    /* 遥控器心跳检测（检查DBUS通信是否正常） */
    rc_sensor.heart_beat(&rc_sensor);

    /* ====== 发射机构控制 ====== */
    if (launcher.info->rc_work_state == DEV_ONLINE)
    {
        /* 遥控器在线：执行正常控制逻辑（摩擦轮速度控制、拨弹控制等） */
        launcher.ctrl();
    }
    else
    {
        /* 遥控器离线：执行自我保护（停止发射、降低摩擦轮转速等） */
        launcher.self_protect();
    }
}


/**
  * @brief  定时器中断回调函数（TIM4，每1ms进入一次）
  * @details HAL库定时器中断回调函数。当TIM4产生更新中断时自动调用。
  *          该回调函数驱动user_main()以1ms为周期执行。
  *
  *          注意事项：
  *          - 在中断上下文中执行，应尽量简短
  *          - 使用静态局部变量维护计数器，避免全局变量污染
  *          - 计数器上限60000（约60秒），防止溢出后行为异常
  *
  * @param  htim : 定时器句柄指针
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)  /* 确认是TIM4的中断 */
    {
        static uint16_t i = 0;  /* 静态计数器，记录中断次数 */

        if (++i == 60000)
        {
            i = 0;  /* 约60秒后重置计数器，防止溢出 */
        }

        /* 调用用户主函数，执行1ms周期控制逻辑 */
        user_main();
    }
}
