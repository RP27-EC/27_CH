/**
  ******************************************************************************
  * @file    board_protocol.c
  * @brief   板间通信协议测试代码
  * @details 本文件实现了板间通信的测试功能：
  *          - 数据包发送（支持结构体和字节数组两种格式）
  *          - 数据包接收和CRC校验
  *          - 支持CRC8和CRC16双重校验
  *          - 支持UART1、UART7、UART10三个串口
  * @note   协议格式：
  *         [帧头 0xA5] [数据...] [CRC8校验] [CRC16校验]
  *         - CRC8校验范围：前3字节（帧头+前2字节数据）
  *         - CRC16校验范围：整个数据包（不含CRC16本身）
  ******************************************************************************
  */

/* 包含头文件 */
#include "board_protocol.h"

/* 外部串口句柄声明 - 用于板间通信的三个串口 */
extern UART_HandleTypeDef huart1;    /* USART1 - 串口1 */
extern UART_HandleTypeDef huart7;    /* UART7 - 串口7 */
extern UART_HandleTypeDef huart10;   /* USART10 - 串口10 */

/**
  * @brief  校验字节数量
  * @details 定义数据包的有效数据长度（不含CRC）
  *          默认值为60字节
  */
int num = 60;

/**
  * @brief  接收数据缓冲区
  * @details 存储通过CRC校验的有效数据
  *          用于观察和验证接收的数据内容
  */
uint8_t rx_info_buff[100];

/**
  * @brief  发送数据缓冲区
  * @details 存储要发送的数据包内容
  */
uint8_t tx_info_buff[100];

/**
  * @brief  CRC计算临时缓冲区
  * @details 用于存储CRC计算过程中的数据
  */
uint8_t crc_tx[60];

/**
  * @brief  CRC校验通过标志
  * @details 标记最近一次接收的数据是否通过CRC校验
  *          true: 校验通过
  *          false: 校验失败
  */
bool is_pass_crc;

/**
  * @brief  测试结构体
  * @details 用于测试结构体方式的数据传输
  *          初始帧头为0xA5
  */
TestStruct test_struct = {
  .u8_1 = 0xA5,    /* 帧头 */
};

/**
  * @brief  发送数据长度变量
  * @details 存储结构体的大小，用于DMA发送
  */
uint32_t lenghth;

/**
  * @brief  板间通信初始化函数
  * @details 初始化发送缓冲区内容：
  *          - 第一个字节为帧头0xA5
  *          - 后续字节为递增的序列号（1, 2, 3, ...）
  * @note   该函数应在系统初始化时调用
  * @retval None
  */
void board_test_init()
{
    tx_info_buff[0] = 0xA5;  /* 设置帧头 */
    for(uint8_t i = 1; i < num; i++)
    {
        tx_info_buff[i] = i + 1;  /* 填充递增数据 */
    }
}

/**
  * @brief  数据包发送函数
  * @details 该函数完成以下工作：
  *          1. 获取结构体大小
  *          2. 递增序列号（用于验证数据连续性）
  *          3. 根据编译选项选择发送模式：
  *             - STRUCT_TEST: 使用结构体方式发送
  *             - 默认: 使用字节数组方式发送
  *          4. 计算CRC8校验（前3字节）
  *          5. 计算CRC16校验（整个数据包）
  *          6. 通过DMA发送数据
  * @note   CRC校验顺序：
  *         - 先计算CRC8：Append_CRC8_Check_Sum(data, 3)
  *         - 再计算CRC16：Append_CRC16_Check_Sum(data, num)
  * @retval None
  */
void board_send()
{
    /* 获取结构体大小 */
    lenghth = sizeof(TestStruct);

    /* 递增序列号，用于验证数据连续性 */
    tx_info_buff[1]++;
    if(tx_info_buff[1] >= 255)
        tx_info_buff[1] = 0;  /* 溢出时回绕到0 */

    #ifdef STRUCT_TEST
    /* 结构体发送模式 */
    test_struct.u8_2 = tx_info_buff[1];  /* 更新结构体字段 */

    /* 计算CRC校验 */
    Append_CRC8_Check_Sum((uint8_t*)&test_struct, 3);      /* CRC8校验前3字节 */
    Append_CRC16_Check_Sum((uint8_t*)&test_struct, lenghth); /* CRC16校验整个结构体 */

    /* 通过DMA发送结构体 */
    HAL_UART_Transmit_DMA(&huart10, (uint8_t*)&test_struct, lenghth);

    #else
    /* 字节数组发送模式 */
    memcpy(crc_tx, tx_info_buff, num);  /* 复制数据到CRC缓冲区 */

    /* 计算CRC校验 */
    Append_CRC8_Check_Sum(crc_tx, 3);    /* CRC8校验前3字节 */
    Append_CRC16_Check_Sum(crc_tx, num); /* CRC16校验整个数据包 */

    /* 通过DMA发送数据 */
    HAL_UART_Transmit_DMA(&huart10, crc_tx, num);

    #endif
}

/**
  * @brief  数据包接收和校验函数
  * @param  rxBuf: 接收到的原始数据缓冲区
  * @details 该函数完成以下工作：
  *          1. 检查输入指针是否为空
  *          2. 验证CRC8校验（前3字节）
  *          3. 验证CRC16校验（整个数据包）
  *          4. 如果校验通过，将有效数据复制到接收缓冲区
  * @note   校验顺序：
  *         - 先验证CRC8：Verify_CRC8_Check_Sum(data, 3)
  *         - 再验证CRC16：Verify_CRC16_Check_Sum(data, num)
  * @retval bool 返回校验结果：
  *              true: 校验通过，数据有效
  *              false: 校验失败或数据无效
  */
bool board_recieve(uint8_t *rxBuf)
{
    /* 空指针检查 */
    if (rxBuf == NULL) {
        return false;
    }

    /* 验证CRC8校验（前3字节） */
    if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
    {
        #ifdef STRUCT_TEST
        /* 结构体模式：验证CRC16校验整个结构体 */
        if(Verify_CRC16_Check_Sum(rxBuf, lenghth) == true)
        {
            memcpy(&rx_info_buff, rxBuf, lenghth);  /* 复制有效数据 */
            return true;
        }
        #else
        /* 字节数组模式：验证CRC16校验整个数据包 */
        if(Verify_CRC16_Check_Sum(rxBuf, num) == true)
        {
            memcpy(&rx_info_buff, rxBuf, num);      /* 复制有效数据 */
            return true;
        }
        #endif
    }
    else return false;  /* CRC8校验失败 */
}

/**
  * @brief  USART1接收数据处理回调函数
  * @param  rxBuf: 接收到的数据缓冲区
  * @details 当USART1接收到完整数据包时调用此函数
  *          调用board_recieve()进行CRC校验
  * @retval None
  */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
    is_pass_crc = board_recieve(rxBuf);
}

/**
  * @brief  UART7接收数据处理回调函数
  * @param  rxBuf: 接收到的数据缓冲区
  * @details 当UART7接收到完整数据包时调用此函数
  *          调用board_recieve()进行CRC校验
  * @retval None
  */
void USART7_rxDataHandler(uint8_t *rxBuf)
{
    is_pass_crc = board_recieve(rxBuf);
}

/**
  * @brief  USART10接收数据处理回调函数
  * @param  rxBuf: 接收到的数据缓冲区
  * @details 当USART10接收到完整数据包时调用此函数
  *          调用board_recieve()进行CRC校验
  * @retval None
  */
void USART10_rxDataHandler(uint8_t *rxBuf)
{
    is_pass_crc = board_recieve(rxBuf);
}
