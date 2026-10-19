/**
  ******************************************************************************
  * @file           : chassis_kinematics.c
  * @brief          : 底盘麦轮运动学解算模块
  */

#include "chassis_kinematics.h"

#include <math.h>

/**
  * @brief  麦克纳姆轮底盘逆运动学解算
  * @details 根据底盘的三自由度运动指令，通过麦轮运动学公式计算四个轮子的目标转速。
  *          输入为归一化的速度分量（[-1, 1]范围），输出为实际RPM转速值。
  *
  *          算法流程：
  *          1. 根据运动学公式计算四个轮子的归一化转速
  *          2. 找出四个轮子转速中的最大绝对值
  *          3. 如果最大值超过1.0，则按比例缩放所有轮速，防止超出电机能力
  *          4. 将归一化值乘以最大RPM得到实际转速
  *
  * @param  vx       : 前后方向速度分量，正值向前，负值向后，范围[-1, 1]
  * @param  vy       : 左右方向速度分量，正值向左，负值向右，范围[-1, 1]
  * @param  wz       : 旋转角速度分量，正值逆时针旋转，负值顺时针旋转，范围[-1, 1]
  * @param  max_rpm  : 电机允许的最大转速(RPM)，用于输出限幅（如M3508为6000RPM）
  * @param  wheel_rpm: 输出数组，存储四个轮子的目标转速(RPM)
  *                    数组顺序：[FL, FR, BL, BR]
  * @retval None
  */
void Chassis_Mecanum_Calculate(float vx, float vy, float wz,
                               float max_rpm,
                               float wheel_rpm[CHASSIS_WHEEL_COUNT])
{
    float max_abs = 0.0f;  /* 四个轮子转速中的最大绝对值，用于归一化 */
    float scale;           /* 归一化缩放系数 */

    /* 参数校验：指针为空时直接返回，防止空指针崩溃 */
    if (wheel_rpm == 0) {
        return;
    }

    /* 参数校验：最大转速无效时，清零所有输出！！！！！并返回 */
    if (max_rpm <= 0.0f) {
        for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
            wheel_rpm[i] = 0.0f;
        }
        return;
    }

    /*=========================================================================
     * 麦轮运动学逆解算：将三自由度速度映射为四个轮子的转速
     * 注意：这里的vx、vy、wz都是归一化后的值，范围在[-1, 1]之间
     *==========================================================================*/
    wheel_rpm[CHASSIS_WHEEL_FL] = vx - vy - wz;  /* 前左轮：前移+左移-旋转 */
    wheel_rpm[CHASSIS_WHEEL_FR] = vx + vy + wz;  /* 前右轮：前移+左移+旋转 */
    wheel_rpm[CHASSIS_WHEEL_BL] = vx + vy - wz;  /* 后左轮：后移+左移-旋转 */
    wheel_rpm[CHASSIS_WHEEL_BR] = vx - vy + wz;  /* 后右轮：后移+左移+旋转 */

    /*=========================================================================
     * 输出限幅处理：找出四个轮子转速中的最大绝对值
     * 如果某个轮子的转速超出[-1, 1]范围，需要等比缩放所有轮速，
     * 确保相对运动方向不变，同时不超过电机能力上限。
     *==========================================================================*/
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        const float magnitude = fabsf(wheel_rpm[i]);//const 确保只读避免后续被修改（功能上等价不加）
        if (magnitude > max_abs) {
            max_abs = magnitude;
        }
    }

    /* 计算缩放系数：
     * - 如果最大绝对值 > 1，按 max_rpm / max_abs 等比缩放，使最大轮速恰好等于max_rpm
     * - 如果最大绝对值 <= 1，直接乘以max_rpm，保留原始速度比例 */
    scale = (max_abs > 1.0f) ? (max_rpm / max_abs) : max_rpm;

    /* 应用缩放系数，将归一化值转换为实际RPM */
    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        wheel_rpm[i] *= scale;
    }
}
