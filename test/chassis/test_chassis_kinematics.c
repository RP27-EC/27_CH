/**
  ******************************************************************************
  * @file    test_chassis_kinematics.c
  * @brief   底盘运动学单元测试文件
  * @details 本文件包含麦轮底盘运动学逆解算法的单元测试：
  *          - 测试静止状态（零输入）
  *          - 测试前进运动
  *          - 测试横向运动（平移）
  *          - 测试旋转运动
  *          - 测试饱和状态（多方向同时运动）
  *          - 测试边界条件（非正最大转速、空指针）
  * @note   该测试程序使用纯C语言编写，可在PC上独立运行
  ******************************************************************************
  */

/* 包含头文件 */
#include "chassis_kinematics.h"  /* 底盘运动学算法头文件 */

#include <math.h>     /* 数学函数（fabsf用于浮点数绝对值） */
#include <stdio.h>    /* 标准输入输出（printf、fprintf） */
#include <stdlib.h>   /* 标准库（exit函数） */

/**
  * @brief 浮点数比较精度阈值
  * @details 用于比较两个浮点数是否相等，考虑浮点数精度误差
  *          如果两个数的差值小于此值，则认为它们相等
  */
#define EPSILON 0.001f

/**
  * @brief 已执行的测试用例计数器
  * @details 记录成功执行的测试用例数量，用于最后输出统计信息
  */
static unsigned int tests_run;

/**
  * @brief  期望车轮速度比较函数
  * @param  name: 测试用例名称（用于错误输出）
  * @param  vx: 期望的X方向速度（前进/后退）
  * @param  vy: 期望的Y方向速度（左移/右移）
  * @param  wz: 期望的角速度（旋转）
  * @param  max_rpm: 最大允许转速
  * @param  expected: 期望的四个车轮转速数组
  * @details 该函数执行以下操作：
  *          1. 调用底盘运动学逆解算法计算实际车轮转速
  *          2. 比较实际转速与期望转速
  *          3. 如果任何一个车轮的误差超过阈值，则报告错误并退出
  * @note   麦轮底盘四个车轮的编号约定：
  *         - 0: 左前轮（FL）
  *         - 1: 右前轮（FR）
  *         - 2: 左后轮（BL）
  *         - 3: 右后轮（BR）
  * @retval None
  */
static void expect_wheels(const char *name,
                          float vx, float vy, float wz, float max_rpm,
                          const float expected[CHASSIS_WHEEL_COUNT])
{
    float actual[CHASSIS_WHEEL_COUNT] = {0.0f};  /* 存储实际计算结果 */

    /* 调用底盘运动学逆解算法 */
    Chassis_Mecanum_Calculate(vx, vy, wz, max_rpm, actual);
    tests_run++;  /* 递增测试计数器 */

    /* 比较每个车轮的实际转速与期望转速 */
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        if (fabsf(actual[i] - expected[i]) >= EPSILON) {
            /* 发现不匹配，输出错误信息并退出 */
            fprintf(stderr, "%s wheel %u: expected %.3f, got %.3f\n",
                    name, i, expected[i], actual[i]);
            exit(EXIT_FAILURE);
        }
    }
}

/**
  * @brief  主函数 - 测试入口
  * @details 该函数执行以下测试用例：
  *          1. zero: 静止状态，所有输入为0
  *          2. forward: 前进运动，X方向速度=1
  *          3. lateral: 横向平移，Y方向速度=1
  *          4. rotate: 旋转运动，角速度=1
  *          5. saturated: 饱和状态，三个方向同时运动
  *          6. non-positive max: 边界测试，最大转速为0
  *          7. NULL指针测试：验证空指针处理
  * @retval int 返回EXIT_SUCCESS表示所有测试通过
  */
int main(void)
{
    /**
      * 定义期望的车轮转速数组
      * 数组索引：0=左前, 1=右前, 2=左后, 3=右后
      */
    const float zero[] = {0.0f, 0.0f, 0.0f, 0.0f};           /* 静止状态 */
    const float forward[] = {6000.0f, 6000.0f, 6000.0f, 6000.0f};  /* 前进 */
    const float lateral[] = {-6000.0f, 6000.0f, 6000.0f, -6000.0f};  /* 横向平移 */
    const float rotate[] = {-6000.0f, 6000.0f, -6000.0f, 6000.0f};   /* 旋转 */
    const float saturated[] = {-2000.0f, 6000.0f, 2000.0f, 2000.0f}; /* 饱和 */

    /* 执行测试用例 */
    expect_wheels("zero", 0.0f, 0.0f, 0.0f, 6000.0f, zero);       /* 测试1：静止 */
    expect_wheels("forward", 1.0f, 0.0f, 0.0f, 6000.0f, forward); /* 测试2：前进 */
    expect_wheels("lateral", 0.0f, 1.0f, 0.0f, 6000.0f, lateral); /* 测试3：横向 */
    expect_wheels("rotate", 0.0f, 0.0f, 1.0f, 6000.0f, rotate);   /* 测试4：旋转 */
    expect_wheels("saturated", 1.0f, 1.0f, 1.0f, 6000.0f, saturated);  /* 测试5：饱和 */
    expect_wheels("non-positive max", 1.0f, 0.0f, 0.0f, 0.0f, zero);   /* 测试6：非正最大值 */

    /* 测试7：空指针测试 - 验证算法不会因空指针崩溃 */
    Chassis_Mecanum_Calculate(1.0f, 0.0f, 0.0f, 6000.0f, NULL);
    tests_run++;

    /* 输出测试结果 */
    printf("%u tests passed\n", tests_run);
    return EXIT_SUCCESS;
}
