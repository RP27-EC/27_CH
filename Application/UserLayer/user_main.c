/**
 * @file    user_main.c
 * @author  SSDCFXB
 * @Version V1.0
 * @date    18-November-2022
 * @brief   用户主程序入口
 *
 * 本文件是用户应用层的主入口，包含：
 * 1. 用户设备初始化函数（USER_Init）
 * 2. 用户主循环函数（user_main），由定时器中断每1ms调用一次
 * 3. 定时器中断回调函数，作为整个用户应用的时基
 *
 * 调用链：TIM4中断(1ms) -> HAL_TIM_PeriodElapsedCallback -> user_main
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"  /* STM32 HAL库头文件 */
#include "tim.h"            /* 定时器头文件 */

#include "user_main.h"      /* 用户主程序头文件 */

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void user_main(void);       /* 用户主循环函数前向声明 */
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  用户设备初始化函数
 * @note   在系统启动时调用，完成以下初始化操作：
 *         1. 初始化所有电机
 *         2. 初始化TIM4定时器（1ms时基）
 *         3. 启动TIM4定时器中断
 *         4. 初始化发射机构
 */
void USER_Init(void)
{
    motor_all_init();                       /* 初始化所有电机 */
    HAL_TIM_Base_Init(&htim4);             /* 初始化TIM4定时器基础配置 */
    HAL_TIM_Base_Start_IT(&htim4);         /* 启动TIM4中断，开始计时 */
    launcher.init();                        /* 初始化发射机构 */
}

/**
 * @brief  用户应用层主循环，每1ms执行一次（由TIM4中断调用）
 * @note   功能：
 *         1. 更新各电机心跳状态，判断是否在线
 *         2. 更新遥控器心跳状态
 *         3. 根据遥控器状态决定发射机构行为：
 *            - 在线：执行正常控制逻辑
 *            - 离线：执行自我保护逻辑
 */
void user_main(void)
{
    /* 更新各电机心跳，用于判断电机是否失联 */
    motor[FRIC_R].heartbeat(&motor[FRIC_R]);   /* 右摩擦轮电机心跳 */
    motor[FRIC_L].heartbeat(&motor[FRIC_L]);   /* 左摩擦轮电机心跳 */
    motor[DIAL].heartbeat(&motor[DIAL]);        /* 拨弹轮电机心跳 */

    /* 更新遥控器心跳状态 */
    rc_sensor.heart_beat(&rc_sensor);

    if (launcher.info->rc_work_state == DEV_ONLINE)
    {
        launcher.ctrl();        /* 遥控器在线：执行正常控制逻辑 */
    }
    else
    {
        launcher.self_protect();  /* 遥控器离线：执行自我保护逻辑，停止发射 */
    }
}

/**
 * @brief  定时器中断回调函数，每1ms进入一次
 * @param  htim: 定时器句柄指针
 * @note   由HAL库自动调用。判断中断来源是否为TIM4，
 *         若是则调用 user_main() 执行用户应用逻辑。
 *         使用静态变量i实现60000次（60秒）的计数器，
 *         可用于长时间定时任务的触发。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)  /* 判断是否为TIM4中断 */
    {
        static uint16_t i = 0;

        if (++i == 60000)  /* 60秒计数器回绕 */
        {
            i = 0;
        }

        user_main();  /* 调用用户主循环函数 */
    }
}

/* 以下是注释掉的备用控制任务代码，供参考 */
// int16_t send_buff[4];
// float tar;
// void StartControlTask(void const * argument)
// {
//     // 电机初始化
//     motor[GIMB_Y].init(&motor[GIMB_Y]);
//     // 电机速度pid初始化
//     motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.speed, gimb_y_speed_pid_param);
//
//     for(;;)
//     {
//         // 电机心跳，用于判断是否失联
//         motor[GIMB_Y].heartbeat(&motor[GIMB_Y]);
//         // pid计算
//         send_buff[motor[GIMB_Y].id.buff_p] = motor[GIMB_Y].c_speed(&motor[GIMB_Y], tar);
//         // 控制数据的发送
//         CAN1_Send_With_int16_to_uint8(motor[GIMB_Y].id.tx_id, send_buff);
//
//         osDelay(1);
//     }
// }
