/**
 * @file    config_uart.c
 * @brief   串口通信配置与数据接收处理模块
 *
 * 本文件实现了各串口（USART）的数据接收回调函数配置。
 * 负责将接收到的串口数据分发给对应的处理模块：
 * - USART1: 预留用于视觉模块通信
 * - USART3: 遥控器数据接收（主遥控器串口）
 * - USART6: 预留用于扩展通信
 */

/* Includes ------------------------------------------------------------------*/
#include "config_uart.h"    /* 串口配置头文件 */
#include "rc_sensor.h"      /* 遥控器传感器驱动 */

/**
 * @brief  USART1接收数据处理函数（视觉模块通信）
 * @param  rxBuf: 接收数据缓冲区指针
 * @note   预留接口，用于接收视觉模块（如摄像头、雷达）发送的数据。
 *         可解析视觉目标位置、识别结果等信息。
 */
// void USART1_rxDataHandler(uint8_t *rxBuf)
// {
//     /* 预留：视觉数据解析逻辑 */
// }

/**
 * @brief  USART3接收数据处理函数（遥控器通信）
 * @param  rxBuf: 接收数据缓冲区指针，包含遥控器原始数据帧
 * @note   USART3连接DJI遥控器接收机，每11ms接收一帧18字节数据。
 *         处理流程：
 *         1. 调用 rc_sensor.update() 解析遥控器协议数据
 *         2. 调用 rc_sensor.check() 检查遥控器连接状态
 */
void USART3_rxDataHandler(uint8_t *rxBuf)
{
    rc_sensor.update(&rc_sensor, rxBuf);  /* 解析遥控器协议数据 */
    rc_sensor.check(&rc_sensor);          /* 检查遥控器状态（离线检测等） */
}

/**
 * @brief  USART6接收数据处理函数（扩展通信）
 * @param  rxBuf: 接收数据缓冲区指针
 * @note   预留接口，可用于：
 *         - 调试串口通信
 *         - 上位机数据交互
 *         - 扩展传感器通信
 */
void USART6_rxDataHandler(uint8_t *rxBuf)
{
    /* 预留：扩展通信数据解析逻辑 */
}
