/**
  ******************************************************************************
  * @file           : judge_protocol.c
  * @author         : czf
  * @date           : 2022-4-22 14:38:38
  * @brief          : 裁判系统（Referee）串口通信协议解析
  * @details        : 本文件实现了RoboMaster裁判系统的串口数据接收与解析协议。
  *                   裁判系统通过串口以自定义帧格式发送比赛数据，帧结构为：
  *                   [帧头0xA5][数据长度+序列号+CRC8][命令字CMD_ID][数据段][CRC16校验]
  *
  *                   本文件主要完成以下功能：
  *                   1. 帧头校验（SOF = 0xA5）
  *                   2. CRC8校验（帧头完整性验证）
  *                   3. CRC16校验（整帧数据完整性验证）
  *                   4. 根据CMD_ID分发到对应的解析函数
  *                   5. 支持一个数据包中包含多帧数据的递归解析
  * @history        :
  ******************************************************************************
  */

#include "judge_protocol.h"
#include "string.h"
#include "crc.h"
#include "judge.h"

/* ==================== 全局变量定义 ==================== */

/** @var judge_frame_header  裁判系统帧头结构体，存储帧头解析后的字段 */
judge_frame_header_t judge_frame_header;

/**
  * @var drv_judge_info  裁判系统驱动信息结构体
  * @details 包含帧头指针、命令字、帧尾等所有解析所需的数据成员
  */
drv_judge_info_t drv_judge_info = {
    .frame_header = &judge_frame_header,  /* 帧头指针指向全局帧头结构体 */
};

/**
  * @brief  裁判系统串口数据接收与解析函数
  * @details 对裁判系统串口发来的原始数据进行逐层校验和解析。
  *          解析流程：
  *          1. 检查帧起始标志 SOF 是否为 0xA5
  *          2. 拷贝帧头5字节（SOF + data_length + seq + CRC8）
  *          3. CRC8校验帧头
  *          4. 计算完整帧长度，进行CRC16校验整帧
  *          5. 提取CMD_ID，调用judge_update()分发到对应处理函数
  *          6. 如果缓冲区中后续还有数据帧，递归调用自身继续解析
  *
  * @param  rxBuf : 串口接收缓冲区指针，包含一帧或多帧裁判系统数据
  * @retval None
  *
  * @note   该函数由串口接收中断/DMA回调触发调用
  * @note   裁判系统帧格式：
  *         Byte 0:      SOF (0xA5) — 帧起始标识
  *         Byte 1-2:    data_length — 数据段长度（低11位）+ 序列号（高5位）
  *         Byte 3:      seq — 帧序列号
  *         Byte 4:      CRC8 — 帧头校验值
  *         Byte 5-6:    CMD_ID — 命令字（标识数据类型）
  *         Byte 7-N:    数据段（长度为data_length）
  *         Byte N+1~N+2:CRC16 — 整帧校验值
  */
void judge_recive(uint8_t *rxBuf)
{
    uint16_t frame_length;

    /* 空指针保护 */
    if( rxBuf == NULL )
    {
        return;
    }

    /* 步骤1：读取帧起始标志字节 SOF（Start Of Frame） */
    drv_judge_info.frame_header->SOF = rxBuf[0];

    /* 步骤2：验证帧起始标志是否为约定值 0xA5 */
    if(drv_judge_info.frame_header->SOF == 0xA5)
    {
        /* 步骤3：拷贝帧头剩余4字节（data_length[2] + seq[1] + CRC8[1]） */
        memcpy(&drv_judge_info.frame_header->data_length, rxBuf + 1, 4);

        /* 步骤4：对帧头5字节进行CRC8校验（验证帧头完整性） */
        if(Verify_CRC8_Check_Sum(rxBuf, 5) == 1)
        {
            /* 步骤5：计算完整帧长度 = 帧头(5) + CMD_ID(2) + 数据段 + CRC16(2) */
            frame_length = 5 + 2 + drv_judge_info.frame_header->data_length + 2;

            /* 步骤6：对整帧数据进行CRC16校验（验证数据完整性） */
            if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == 1)
            {
                /* 步骤7：提取CMD_ID（2字节），拷贝帧头之后的内容 */
                memcpy(&drv_judge_info.cmd_id, rxBuf + 5, 2);

                /* 步骤8：根据CMD_ID将数据分发到对应的解析处理函数 */
                judge_update(drv_judge_info.cmd_id, rxBuf + 7);
            }

            /* 提取帧尾CRC16校验值 */
            memcpy(&drv_judge_info.frame_tail, rxBuf + 5 + 2 + drv_judge_info.frame_header->data_length, 2);

            /* 步骤9：处理一个数据包中包含多帧数据的情况
             * 如果当前帧之后紧接的字节仍是 0xA5，则说明后面还有数据帧，
             * 递归调用自身，从下一帧的起始位置继续解析 */
            if(rxBuf[frame_length] == 0xA5)
            {
                judge_recive( &rxBuf[frame_length] );
            }
        }
    }
}

/**
  * @brief USART1 接收回调重写：把串口数据桥接给裁判系统解析
  */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
    judge_recive(rxBuf);
}
