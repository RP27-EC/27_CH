/**
  ******************************************************************************
  * @file           : ave_filter.c
  * @brief          : 滑动均值滤波器实现
  * @details        : 实现了一种高效的滑动窗口均值滤波算法。该算法通过维护一个固定大小
  *                   的环形缓冲区，对连续输入的数据进行均值计算，从而实现信号平滑处理。
  *                   采用增量式更新策略，每次更新只需做一次减法和一次加法，计算效率高。
  *
  * @note           :
  *                  - 最大滤波窗口大小由 ave_filter_times_max 宏定义（默认30）
  *                  - 支持动态调整滤波窗口大小
  *                  - 适用于传感器数据平滑、信号去噪等场景
  * @date            : 2022-1-21 15:10:55
  ******************************************************************************
  */

/* 包含头文件 ------------------------------------------------------------------*/
#include "ave_filter.h" /* 均值滤波器头文件，包含结构体定义和接口声明 */
#include "string.h"    /* 字符串/内存操作库，用于memset */

/**
  * @brief  初始化均值滤波器
  * @details 将滤波器的所有成员变量清零，包括：
  *          - 环形缓冲区数据清零
  *          - 均值结果清零
  *          - 当前索引位置清零
  *          - 滤波次数清零
  * @param  ave_fil: 指向均值滤波器结构体的指针
  * @retval None
  */
void ave_fil_init(ave_filter_t *ave_fil)
{
    /* 将环形缓冲区所有元素清零 */
    memset(ave_fil->value, 0, ave_filter_times_max);
    ave_fil->value_ave = 0;      /* 均值结果清零 */
    ave_fil->index = 0;          /* 当前写入索引清零 */
    ave_fil->filter_times = 0;   /* 滤波窗口大小清零 */
}

/**
  * @brief  更新均值滤波器并返回滤波结果
  * @details 实现滑动窗口均值滤波的核心算法：
  *          1. 检查并限制最大滤波次数不超过缓冲区大小
  *          2. 如果滤波次数发生变化，重新初始化滤波器
  *          3. 增量式更新：减去即将被覆盖的旧值，加上新值
  *          4. 循环索引实现环形缓冲区
  *
  * @param  ave_fil: 指向均值滤波器结构体的指针
  * @param  value:   当前采样值（新输入的数据）
  * @param  max:     滤波窗口大小（参与均值计算的数据个数）
  * @retval float:   滤波后的均值结果
  */
float ave_fil_update(ave_filter_t *ave_fil, float value, uint16_t max)
{
    /* 限制滤波次数不超过最大缓冲区大小，防止数组越界 */
    if(max > ave_filter_times_max)
    {
        max = ave_filter_times_max;
    }

    /* 如果滤波窗口大小发生变化，重新初始化滤波器 */
    if(max != ave_fil->filter_times)
    {
        ave_fil_init(ave_fil);          /* 清零所有数据 */
        ave_fil->filter_times = max;    /* 记录新的滤波窗口大小 */
    }

    /* ===== 增量式滑动均值计算 ===== */
    /* 步骤1: 从当前均值中减去即将被覆盖的旧值的贡献 */
    ave_fil->value_ave -= ave_fil->value[ave_fil->index] / (float)max;

    /* 步骤2: 将新值写入环形缓冲区的当前位置 */
    ave_fil->value[ave_fil->index] = value;

    /* 步骤3: 将新值的贡献加到当前均值中 */
    ave_fil->value_ave += ave_fil->value[ave_fil->index] / (float)max;

    /* 步骤4: 索引递增，实现环形缓冲区（取模运算） */
    ave_fil->index++;
    ave_fil->index %= max;

    /* 返回当前的滑动均值结果 */
    return ave_fil->value_ave;
}
