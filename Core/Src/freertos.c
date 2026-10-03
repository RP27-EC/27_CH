/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS应用程序代码
  * @details           本文件包含FreeRTOS操作系统的初始化和任务创建代码。
  *                    系统使用CMSIS-OS2接口，创建了5个任务：
  *                    1. MonitorTask - 监控任务（高优先级）
  *                    2. CtrlTask - 控制任务（高优先级）
  *                    3. CommandTask - 命令处理任务（最高优先级）
  *                    4. UpdataTask - 数据更新任务（低优先级）
  *                    5. UITask - 用户界面任务（低优先级）
  ******************************************************************************
  * @attention
  * 版权所有 (c) 2025 STMicroelectronics。保留所有权利。
  ******************************************************************************
  */

/* 包含头文件 */
#include "FreeRTOS.h"      /* FreeRTOS内核头文件 */
#include "task.h"          /* 任务管理头文件 */
#include "main.h"          /* 主程序头文件 */
#include "cmsis_os.h"      /* CMSIS-OS2接口头文件 */

/**
  * ============================================================================
  * 任务句柄和属性定义
  * ============================================================================
  * 每个任务都需要一个句柄（用于任务管理）和属性结构体（定义任务参数）
  */

/**
  * @brief 监控任务配置
  * @details 用于系统状态监控、看门狗喂狗、系统健康检查等
  */
osThreadId_t MonitorTaskHandle;  /* 监控任务句柄 */
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",               /* 任务名称 */
  .stack_size = 512 * 4,              /* 栈大小：2048字节（512*4） */
  .priority = (osPriority_t) osPriorityAboveNormal,  /* 优先级：高于普通 */
};

/**
  * @brief 控制任务配置
  * @details 用于运动控制、电机控制、PID计算等实时性要求高的控制逻辑
  */
osThreadId_t CtrlTaskHandle;    /* 控制任务句柄 */
const osThreadAttr_t CtrlTask_attributes = {
  .name = "CtrlTask",                 /* 任务名称 */
  .stack_size = 512 * 4,              /* 栈大小：2048字节 */
  .priority = (osPriority_t) osPriorityAboveNormal,  /* 优先级：高于普通 */
};

/**
  * @brief 命令处理任务配置
  * @details 用于处理上位机指令、遥控器数据解析、命令分发等
  *          优先级最高，确保命令能及时响应
  */
osThreadId_t CommandTaskHandle; /* 命令处理任务句柄 */
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",              /* 任务名称 */
  .stack_size = 256 * 4,              /* 栈大小：1024字节 */
  .priority = (osPriority_t) osPriorityHigh,  /* 优先级：高（最高） */
};

/**
  * @brief 数据更新任务配置
  * @details 用于传感器数据读取、数据处理、状态更新等非实时性任务
  */
osThreadId_t UpdataTaskHandle;  /* 数据更新任务句柄 */
const osThreadAttr_t UpdataTask_attributes = {
  .name = "UpdataTask",               /* 任务名称 */
  .stack_size = 256 * 4,              /* 栈大小：1024字节 */
  .priority = (osPriority_t) osPriorityBelowNormal,  /* 优先级：低于普通 */
};

/**
  * @brief 用户界面任务配置
  * @details 用于OLED显示、LED控制、按键处理等人机交互功能
  *          栈空间最大，因为UI操作可能需要较多临时变量
  */
osThreadId_t UITaskHandle;       /* UI任务句柄 */
const osThreadAttr_t UITask_attributes = {
  .name = "UITask",                  /* 任务名称 */
  .stack_size = 1024 * 4,            /* 栈大小：4096字节（最大） */
  .priority = (osPriority_t) osPriorityBelowNormal,  /* 优先级：低于普通 */
};

/* 任务函数原型声明 */
void StartMonitorTask(void *argument);   /* 监控任务入口函数 */
void StartCtrlTask(void *argument);      /* 控制任务入口函数 */
void StartCommandTask(void *argument);   /* 命令处理任务入口函数 */
void StartUpdataTask(void *argument);    /* 数据更新任务入口函数 */
void StartUITask(void *argument);        /* UI任务入口函数 */

/**
  * @brief  FreeRTOS初始化函数
  * @details 该函数完成以下工作：
  *          1. 创建互斥锁（Mutex）- 用于保护共享资源
  *          2. 创建信号量（Semaphore）- 用于任务间同步
  *          3. 创建软件定时器（Timer）- 用于周期性任务
  *          4. 创建消息队列（Queue）- 用于任务间通信
  *          5. 创建所有任务
  * @note   任务优先级说明（数值越大优先级越高）：
  *         - osPriorityHigh: 命令处理任务（最高）
  *         - osPriorityAboveNormal: 监控任务、控制任务（较高）
  *         - osPriorityBelowNormal: 数据更新任务、UI任务（较低）
  * @retval None
  */
void MX_FREERTOS_Init(void) {

  /* 创建互斥锁 - 用于保护共享资源，防止多任务同时访问 */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* 创建信号量 - 用于任务间同步和事件通知 */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* 创建软件定时器 - 用于周期性任务或超时处理 */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* 创建消息队列 - 用于任务间数据传递 */
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* 创建所有任务 */

  /* 创建监控任务 - 负责系统监控、看门狗喂狗等 */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* 创建控制任务 - 负责运动控制、电机控制等实时任务 */
  CtrlTaskHandle = osThreadNew(StartCtrlTask, NULL, &CtrlTask_attributes);

  /* 创建命令处理任务 - 负责处理上位机指令 */
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);

  /* 创建数据更新任务 - 负责传感器数据读取和处理 */
  UpdataTaskHandle = osThreadNew(StartUpdataTask, NULL, &UpdataTask_attributes);

  /* 创建UI任务 - 负责用户界面显示和交互 */
  UITaskHandle = osThreadNew(StartUITask, NULL, &UITask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/**
  * @brief  监控任务入口函数
  * @param  argument: 任务参数（未使用）
  * @details 该任务负责：
  *          - 系统状态监控
  *          - 看门狗喂狗
  *          - 系统健康检查
  *          - 故障检测和处理
  * @note   使用__weak关键字，允许用户在其他文件中重写此函数
  * @retval None
  */
__weak void StartMonitorTask(void *argument)
{
  /* 无限循环 - 任务函数不能返回 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统时钟节拍，让出CPU给其他任务 */
  }
}

/**
  * @brief  控制任务入口函数
  * @param  argument: 任务参数（未使用）
  * @details 该任务负责：
  *          - 运动控制算法执行
  *          - 电机PID控制
  *          - 云台控制
  *          - 底盘控制
  * @note   实时性要求高，需要快速响应
  * @retval None
  */
__weak void StartCtrlTask(void *argument)
{
  /* 无限循环 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统时钟节拍 */
  }
}

/**
  * @brief  命令处理任务入口函数
  * @param  argument: 任务参数（未使用）
  * @details 该任务负责：
  *          - 接收和解析上位机指令
  *          - 遥控器数据处理
  *          - 命令分发和执行
  *          - 协议解析
  * @note   优先级最高，确保命令能及时处理
  * @retval None
  */
__weak void StartCommandTask(void *argument)
{
  /* 无限循环 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统时钟节拍 */
  }
}

/**
  * @brief  数据更新任务入口函数
  * @param  argument: 任务参数（未使用）
  * @details 该任务负责：
  *          - 传感器数据读取（IMU、编码器等）
  *          - 数据滤波和处理
  *          - 状态变量更新
  *          - 非实时数据处理
  * @note   优先级较低，可以被高优先级任务抢占
  * @retval None
  */
__weak void StartUpdataTask(void *argument)
{
  /* 无限循环 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统时钟节拍 */
  }
}

/**
  * @brief  UI任务入口函数
  * @param  argument: 任务参数（未使用）
  * @details 该任务负责：
  *          - OLED/LCD显示更新
  *          - LED状态控制
  *          - 按键扫描和处理
  *          - 蜂鸣器控制
  *          - 人机交互界面
  * @note   栈空间最大（4096字节），因为UI操作可能需要较多临时变量
  * @retval None
  */
__weak void StartUITask(void *argument)
{
  /* 无限循环 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统时钟节拍 */
  }
}
