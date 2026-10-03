/**
 * @file    BMI088Middleware.c
 * @brief   BMI088 硬件抽象层（中间件）
 *
 * 本文件提供了 BMI088 驱动与 STM32 硬件之间的抽象层，包括：
 *   1. GPIO 初始化（片选引脚配置）
 *   2. SPI 通信初始化
 *   3. 微秒级延时函数（基于硬件定时器）
 *   4. SPI 片选控制函数
 *   5. SPI 字节读写函数
 *
 * 该层将硬件操作封装为统一接口，使 BMI088 驱动层不直接依赖具体硬件，
 * 方便移植到不同平台。
 *
 * @note   需要在 CubeMX 中配置 SPI1 和 TIM2
 * @version V1.0
 */

/* Includes ------------------------------------------------------------------*/
#include "BMI088Middleware.h"  /* 中间件头文件，包含函数声明 */
#include "main.h"              /* STM32 HAL 库主头文件 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;  /* SPI1 外设句柄（在 CubeMX 中配置） */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  BMI088 GPIO 初始化
 *         配置加速度计和陀螺仪的片选引脚
 * @note   当前为空函数，GPIO 初始化在 CubeMX 中完成
 *         如需动态配置 GPIO，可在此函数中添加配置代码
 */
void BMI088_GPIO_init(void)
{
    /* GPIO 初始化已在 CubeMX 中配置完成 */
}

/**
 * @brief  BMI088 通信接口初始化
 *         配置 SPI 通信参数
 * @note   当前为空函数，SPI 初始化在 CubeMX 中完成
 *         如需动态调整 SPI 参数，可在此函数中添加配置代码
 */
void BMI088_com_init(void)
{
    /* SPI 通信已在 CubeMX 中配置完成 */
}

/* Private variables ---------------------------------------------------------*/
/**
 * @brief 微秒计时器的毫秒部分缓存
 * 用于 bmi2_micros() 函数中避免重复调用 HAL_GetTick()
 */
uint32_t btick = 0;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  获取微秒级时间戳
 *         结合 HAL_GetTick()（毫秒级）和 TIM2 计数器（微秒级）实现高精度计时
 * @return 当前时间戳（微秒）
 *
 * @note   算法原理：
 *           - HAL_GetTick() 提供毫秒级基准（每 1ms 递增）
 *           - TIM2->CNT 提供毫秒内的微秒级细分
 *           - 总时间 = 毫秒 * 1000 + 微秒计数
 *         TIM2 需配置为 1MHz 频率（1us 计数一次）
 */
uint32_t bmi2_micros(void)
{
    register uint32_t bms, bus;  /* 寄存器变量：毫秒和微秒部分 */

    bms = HAL_GetTick();  /* 获取系统毫秒计数 */
    bus = TIM2->CNT;      /* 获取 TIM2 当前计数值（毫秒内细分） */

    /* 组合为微秒时间戳 */
    btick = bms*1000 + bus;

    return btick;
}

/**
 * @brief  毫秒级延时函数
 * @param  ms 延时毫秒数
 *
 * @note   通过循环调用微秒延时实现，精度受调用开销影响
 *         建议在需要较长延时（>1ms）时使用
 */
void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);  /* 每次延时 1000us = 1ms */
    }
}

/**
 * @brief  微秒级延时函数
 * @param  us 延时微秒数
 *
 * @note   基于 TIM2 硬件计数器实现，精度较高
 *         原理：记录当前时间戳，循环等待直到经过指定微秒数
 *         注意：延时时间较长时会有累积误差
 */
void BMI088_delay_us(uint16_t us)
{
    uint32_t now = bmi2_micros();  /* 记录开始时间戳 */

    /* 忙等待：循环直到经过指定微秒数 */
    while((bmi2_micros() - now) < us);
}

/**
 * @brief  加速度计 SPI 片选拉低（选中加速度计）
 *
 * @note   BMI088 加速度计和陀螺仪共用 SPI 总线，
 *         通过不同的片选引脚区分通信对象
 */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  加速度计 SPI 片选拉高（释放加速度计）
 */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

/**
 * @brief  陀螺仪 SPI 片选拉低（选中陀螺仪）
 */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  陀螺仪 SPI 片选拉高（释放陀螺仪）
 */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/**
 * @brief  SPI 字节读写函数
 * @param  txdata 要发送的数据字节
 * @return 接收到的数据字节
 *
 * @note   SPI 是全双工通信，发送和接收同时进行
 *         使用 HAL 库的阻塞式传输函数
 *         超时时间设置为 1000ms
 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;  /* 接收缓冲区 */
    /* 阻塞式 SPI 收发：发送 1 字节，同时接收 1 字节 */
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;  /* 返回接收到的数据 */
}
