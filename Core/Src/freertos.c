/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS应用程序代码
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"    /* FreeRTOS核心头文件 */
#include "task.h"        /* 任务管理头文件 */
#include "main.h"        /* 主程序头文件 */
#include "cmsis_os.h"    /* CMSIS-RTOS包装层头文件 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/**
 * 任务句柄声明
 * 每个任务句柄用于标识和控制对应的RTOS任务
 */
osThreadId MonitorTaskHandle;    /* 监控任务句柄 */
osThreadId CommunityTaskHandle;  /* 通信任务句柄 */
osThreadId ControlTaskHandle;    /* 控制任务句柄 */
osThreadId LedTaskHandle;        /* LED任务句柄 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* 任务函数原型声明 */
void StartMonitorTask(void const * argument);    /* 监控任务函数 */
void StartCommunityTask(void const * argument);  /* 通信任务函数 */
void StartControlTask(void const * argument);    /* 控制任务函数 */
void StartLedTask(void const * argument);        /* LED任务函数 */

extern void MX_USB_DEVICE_Init(void);  /* USB设备初始化外部声明 */
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  获取空闲任务内存的原型（链接到静态分配支持）
  * @note   FreeRTOS需要为IDLE任务分配内存，此函数提供静态分配的内存
  * @param  ppxIdleTaskTCBBuffer: 空闲任务TCB缓冲区指针的指针
  * @param  ppxIdleTaskStackBuffer: 空闲任务栈缓冲区指针的指针
  * @param  pulIdleTaskStackSize: 空闲任务栈大小的指针
  * @retval None
  */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
/**
 * 静态分配空闲任务内存
 * 使用静态变量存储空闲任务的TCB和栈，避免动态内存分配
 */
static StaticTask_t xIdleTaskTCBBuffer;  /* 空闲任务TCB缓冲区 */
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];  /* 空闲任务栈 */

/**
  * @brief  获取空闲任务内存的实现
  * @note   将静态分配的TCB和栈缓冲区地址传递给FreeRTOS
  * @param  ppxIdleTaskTCBBuffer: 输出参数，返回空闲任务TCB地址
  * @param  ppxIdleTaskStackBuffer: 输出参数，返回空闲任务栈地址
  * @param  pulIdleTaskStackSize: 输出参数，返回空闲任务栈大小
  * @retval None
  */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;  /* 设置TCB缓冲区地址 */
  *ppxIdleTaskStackBuffer = &xIdleStack[0];     /* 设置栈缓冲区地址 */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;  /* 设置栈大小 */
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS初始化函数
  * @note   创建所有需要的任务，配置RTOS对象（互斥量、信号量、定时器、队列等）
  *         此函数在启动调度器之前调用
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */  /* 在此添加互斥量 */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */  /* 在此添加信号量 */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */  /* 在此启动定时器，添加新的定时器 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */  /* 在此添加队列 */
  /* USER CODE END RTOS_QUEUES */

  /**
    * 创建任务
    * 使用osThreadDef宏定义任务，然后使用osThreadCreate创建任务
    * 每个任务需要指定：任务名、任务函数、优先级、实例数、栈大小
    */

  /* 定义并创建MonitorTask（监控任务）
   * 优先级：osPriorityRealtime（实时优先级）
   * 栈大小：512字（2048字节）
   */
  osThreadDef(MonitorTask, StartMonitorTask, osPriorityRealtime, 0, 512);
  MonitorTaskHandle = osThreadCreate(osThread(MonitorTask), NULL);

  /* 定义并创建CommunityTask（通信任务）
   * 优先级：osPriorityHigh（高优先级）
   * 栈大小：512字（2048字节）
   */
  osThreadDef(CommunityTask, StartCommunityTask, osPriorityHigh, 0, 512);
  CommunityTaskHandle = osThreadCreate(osThread(CommunityTask), NULL);

  /* 定义并创建ControlTask（控制任务）
   * 优先级：osPriorityRealtime（实时优先级）
   * 栈大小：1024字（4096字节）
   */
  osThreadDef(ControlTask, StartControlTask, osPriorityRealtime, 0, 1024);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* 定义并创建LedTask（LED任务）
   * 优先级：osPriorityAboveNormal（高于正常优先级）
   * 栈大小：128字（512字节）
   */
  osThreadDef(LedTask, StartLedTask, osPriorityAboveNormal, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */  /* 在此添加更多任务 */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
  * @brief  监控任务函数
  * @note   负责系统监控，包括USB设备初始化
  *         使用__weak关键字，允许用户重写此函数
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_StartMonitorTask */
__weak void StartMonitorTask(void const * argument)
{
  /* 初始化USB设备 */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartMonitorTask */
  /* 无限循环 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统滴答（通常1ms） */
  }
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartCommunityTask */
/**
* @brief 通信任务函数
* @note  负责处理通信相关功能
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartCommunityTask */
__weak void StartCommunityTask(void const * argument)
{
  /* USER CODE BEGIN StartCommunityTask */
  /* 无限循环 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统滴答 */
  }
  /* USER CODE END StartCommunityTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief 控制任务函数
* @note  负责系统控制逻辑
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartControlTask */
__weak void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* 无限循环 */
  for(;;)
  {
    osDelay(1);  /* 延时1个系统滴答 */
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief LED任务函数
* @note  负责控制LED状态指示
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartLedTask */
__weak void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* 无限循环 */
  for(;;)
  {

    osDelay(1);  /* 延时1个系统滴答 */
  }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */