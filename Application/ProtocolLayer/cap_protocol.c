/**
  ******************************************************************************
  * @file           : cap_protocol.c
  * @author         : czf
  * @date           : 2022-4-22 15:41:14
  * @brief          : 超级电容（CAP）CAN通信协议实现
  * @details        : 本文件实现了与超级电容模块之间的CAN通信协议，包括：
  *                   1. 向电容模块发送底盘功率限制、缓冲能量等控制帧（ID: 0x222）
  *                   2. 接收并解析电容模块返回的状态数据（电压、电流、功率等）
  *                   3. 提供 int16 与 float 之间的双向线性映射工具函数
  * @history        :
  ******************************************************************************
  */

#include "cap_protocol.h"
#include "string.h"
#include "cap.h"

/* ==================== 全局变量定义 ==================== */

/** @var cap_receive_data  电容接收数据结构体（完整帧数据） */
cap_receive_data_t cap_receive_data;

/** @var cap_rx_info       电容接收信息结构体（解析后的字段） */
cap_rx_info_t cap_rx_info;

/** @var cap_tx_info       电容发送信息结构体（控制帧数据） */
cap_transmit_data_t cap_tx_info;

/** @var cap_tx_buf        CAN发送缓冲区，用于存放待发送的8字节数据帧 */
uint8_t cap_tx_buf[8]; // CAN ID: 0x222

/**
  * @brief  向超级电容模块发送控制信息帧
  * @details 将底盘功率缓冲值、功率限制值、充放电功率限制等参数
  *          打包为CAN数据帧，通过FDCAN3总线发送到电容模块（ID: 0x222）。
  *
  * @note   超电板当前为「只读调试」模式：不需要使用超电，只要 debug 时
  *          能读到 cap.info / cap.status 即可。TX 路径整体停用，
  *          函数体注释保留；power_model 模块已删除，恢复供电策略时
  *          需一并重建功率模型并取消下方注释。
  * @retval None
  */
void cap_send_2E(void)
{
    /* ====== 超电 TX 停用（只读调试模式）：恢复时取消注释 ======
    cap_tx_info.chassis_power_buffer = (uint8_t)Power_Budget_Buffer();
    cap_tx_info.chassis_power_limit = (uint16_t)Power_Budget_Get();
    cap_tx_info.cap_power_in_limit = 300;
    cap_tx_info.cap_power_out_limit = -300;
    cap_tx_info.bit_control.turbo_mode = 0;
    cap_tx_info.bit_control.cap_switch = (cap.Y_O_N != 0U) ? 1U : 0U;
    memcpy(cap_tx_buf, &cap_tx_info, sizeof(cap_transmit_data_t));
    CAN_SendData(&hfdcan3, CAP_TX_ID, cap_tx_buf);
    ====== */
}


/**
  * @brief  解析超级电容模块返回的CAN数据并更新电容状态
  * @details 从电容模块的CAN应答帧中提取底盘功率、电容电流、电容电压、
  *          充电能力等级等信息，并更新到全局电容状态结构体中。
  *
  * @param  my_cap  : 电容模块对象指针，指向存储电容状态信息的结构体
  * @param  rxBuf   : CAN接收缓冲区，包含电容模块返回的8字节原始数据
  * @retval None
  */
void cap_update(cap_t* my_cap, uint8_t *rxBuf)
{
    /* 将CAN原始数据拷贝到解析结构体中 */
    memcpy(&cap_rx_info, rxBuf, sizeof(cap_rx_info_t));

    /* 解析并更新：底盘实时功率（单位：W） */
    my_cap->info->chassis_power = cap_rx_info.now_chassis_power;

    /* 解析并更新：电容电流（原始int16映射到[-16, 16]A范围） */
    my_cap->info->cap_I = int16_to_float(cap_rx_info.now_cap_I, 32000, -32000, 16, -16);

    /* 解析并更新：电容电压（原始int16映射到[0, 25]V范围） */
    my_cap->info->cap_Ucr = int16_to_float(cap_rx_info.now_cap_V, 32000, -32000, 25, 0);

    /* 解析并更新：充电能力等级（0/1/2/3对应不同充电功率） */
    my_cap->info->ability = cap_rx_info.bit_state.ability;

    /* 收到有效数据，重置离线计数器（防止被判定为离线） */
    cap.status->offline_cnt = 0;
}

/**
  * @brief  int16整数线性映射到浮点范围
  * @details 将int16类型整数值从[a_min, a_max]范围线性映射到[b_min, b_max]浮点范围。
  *          公式：result = (a - a_min) / (a_max - a_min) * (b_max - b_min) + b_min
  *          常用于将CAN协议中传输的整数原始值还原为实际物理量。
  *
  * @param  a     : 待转换的int16整数值
  * @param  a_max : 源范围最大值（对应b_max）
  * @param  a_min : 源范围最小值（对应b_min）
  * @param  b_max : 目标范围最大值
  * @param  b_min : 目标范围最小值
  * @retval 转换后的float值，范围[b_min, b_max]
  */
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    int32_t a_32 = a, a_max_32 = a_max, a_min_32 = a_min;
    int32_t diff_a = a_max_32 - a_min_32;

    /* 防止除零：若源范围为0则返回中间值 */
    if (diff_a == 0) return (b_max + b_min) / 2.0f;

    /* 计算归一化比例 [0, 1] */
    float ratio = (float)(a_32 - a_min_32) / (float)diff_a;

    /* 线性映射到目标范围 */
    return ratio * (b_max - b_min) + b_min;
}

/**
  * @brief  浮点数线性映射到int16整数范围
  * @details 将float类型的浮点值从[b_min, b_max]范围线性映射到[a_min, a_max]整数范围。
  *          是int16_to_float的逆运算，常用于将物理量转换为CAN协议传输格式。
  *
  * @param  b     : 待转换的float浮点值
  * @param  b_max : 源范围最大值（对应a_max）
  * @param  b_min : 源范围最小值（对应a_min）
  * @param  a_max : 目标int16范围最大值
  * @param  a_min : 目标int16范围最小值
  * @retval 转换后的int16值，范围[a_min, a_max]
  */
int16_t float_to_int16(float b, float b_max, float b_min, int16_t a_max, int16_t a_min)
{
    /* 防止除零：若源范围为0则返回中间值 */
    if (b_max == b_min) return (int16_t)((a_max + a_min) / 2);

    /* 计算归一化比例 [0, 1] */
    float ratio = (b - b_min) / (b_max - b_min);

    /* 映射到目标整数范围，并四舍五入 */
    int32_t a = (int32_t)(ratio * (a_max - a_min) + a_min + 0.5f);

    /* 钳位保护：确保输出不超出目标范围 */
    a = (a < a_min) ? a_min : (a > a_max) ? a_max : a;

    return (int16_t)a;
}
