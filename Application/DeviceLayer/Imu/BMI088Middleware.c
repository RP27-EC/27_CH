/**
  ******************************************************************************
  * @file           : BMI088Middleware.c
  * @brief          : BMI088 IMU传感器的中间件层（硬件抽象层）
  * @details        : 封装了BMI088传感器所需的底层硬件操作，包括：
  *                   - GPIO片选控制（加速度计/陀螺仪）
  *                   - SPI通信读写接口
  *                   - 微秒/毫秒级延时函数
  *                   - 高精度时间戳获取（基于HAL定时器+TIM2组合）
  ******************************************************************************
  */

#include "BMI088Middleware.h"
#include "main.h"

/* 外部声明SPI2句柄，由STM32CubeMX自动生成并初始化 */
extern SPI_HandleTypeDef hspi2;

/**
  * @brief  BMI088 GPIO初始化
  * @note   当前为空函数，GPIO配置在CubeMX中完成，此处预留扩展接口
  */
void BMI088_GPIO_init(void)
{
	/* GPIO初始化已在CubeMX中配置，无需额外操作 */
}

/**
  * @brief  BMI088通信接口初始化
  * @note   当前为空函数，SPI配置在CubeMX中完成，此处预留扩展接口
  */
void BMI088_com_init(void)
{
	/* SPI通信初始化已在CubeMX中配置，无需额外操作 */
}

/* 全局微秒级时间戳缓存变量 */
uint32_t btick = 0;

/**
  * @brief  获取微秒级高精度时间戳
  * @retval 当前时间戳（单位：微秒）
  * @details 组合HAL_GetTick()（毫秒级）和TIM2定时器计数器（微秒级），
  *          实现微秒级精度的时间戳。计算公式：时间戳 = 毫秒值*1000 + TIM2计数
  * @note   TIM2计数器的时钟分频需配置为1MHz（即1us计数一次）
  */
uint32_t bmi2_micros(void)
{
	/* 使用register关键字建议编译器将变量放入寄存器以提高访问速度 */
	register uint32_t bms, bus;

	/* 获取HAL系统滴答定时器的毫秒值 */
	bms = HAL_GetTick();

	/* 获取TIM2定时器的当前计数值（微秒部分） */
	bus = TIM2->CNT;

	/* 组合毫秒和微秒部分，得到完整的微秒时间戳 */
	btick = bms*1000 + bus;

	return btick;
}

/**
  * @brief  毫秒级延时函数
  * @param  ms: 延时时间，单位为毫秒
  * @note   通过多次调用微秒延时实现，精度为1ms
  */
void BMI088_delay_ms(uint16_t ms)
{
    /* 循环ms次，每次延时1000us = 1ms */
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

/**
  * @brief  微秒级延时函数
  * @param  us: 延时时间，单位为微秒
  * @details 记录起始时间戳，通过忙等待（busy-wait）方式延时到目标时间
  * @note   该延时为阻塞式延时，延时期间CPU空转
  */
void BMI088_delay_us(uint16_t us)
{
	/* 记录延时开始时的时间戳 */
	uint32_t now = bmi2_micros();

	/* 忙等待，直到经过指定的微秒数 */
	while((bmi2_micros() - now) < us);
}

/**
  * @brief  加速度计片选信号拉低（选中芯片，准备通信）
  * @note   SPI通信时，片选信号CS为低电平有效
  */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  加速度计片选信号拉高（释放芯片，结束通信）
  */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

/**
  * @brief  陀螺仪片选信号拉低（选中芯片，准备通信）
  */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  陀螺仪片选信号拉高（释放芯片，结束通信）
  */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/**
  * @brief  SPI单字节读写函数
  * @param  txdata: 要发送的字节数据
  * @retval 接收到的字节数据
  * @details 同时发送和接收一个字节，是SPI全双工通信的基础操作。
  *          写操作时返回值可忽略，读操作时返回值为从设备返回的数据。
  * @note   超时时间设置为1000ms，SPI时钟频率需确保在此超时内完成传输
  */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;  /* 存储接收到的数据 */
    /* 使用HAL库SPI发送接收函数，同时发送和接收1个字节 */
    HAL_SPI_TransmitReceive(&hspi2, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

