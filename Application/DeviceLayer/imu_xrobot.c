/**
 ******************************************************************************
 * @file        imu_xrobot.c
 * @author      RobotMaster C Board
 * @brief       ATOM-IMU 成品陀螺仪 UART 通信协议层
 *
 * 本文件实现与 XRobot ATOM-IMU V5.3 成品陀螺仪的 UART 通信协议。
 *
 * ==== 通信方式 ====
 *
 * 通过 USART1 + DMA 循环模式 + 空闲中断 接收数据。
 * DRV_UART_IRQHandler() 检测到 USART1 空闲中断后，
 * 调用 USART1_rxDataHandler() 进行帧解析。
 * 解析成功的数据存入全局变量 imu_data。
 *
 * ==== 使用流程 ====
 *
 * 1. main.c 中已调用 DEVICE_Init() -> IMU_UART_Init()，完成初始化。
 * 2. 数据自动在 USART1 空闲中断中接收、校验、解析。
 * 3. 直接读取 imu_data 全局变量即可获得最新数据。
 *    见 imu_xrobot.h 中的示例代码。
 *
 * ==== 注意事项 ====
 *
 * - USART1 波特率需配置为 2000000，需要在usart.c中确认。
 * - DMA 接收缓冲区需足够大，即USART1_RX_BUF_LEN = 128，drv_uart.c和drv_uart.h中都有定义，删去其中一个并改为128即可。
 * - 需要在drv_uart.c的uart_rx_idle_callback函数中uart1的部分else if (huart == &huart1)内添加__HAL_DMA_SET_COUNTER(huart->hdmarx, USART1_RX_BUF_LEN)让DMA重启时重置计数器
 * - 需要在drv_uart.h里面添加extern DMA_HandleTypeDef hdma_usart1_rx;
 * - 删掉config_uart.c里面的USART1_rxDataHandler
 * @version V1.0
 * @date    2026-06-11
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "imu_xrobot.h"    /* IMU UART 协议头文件，包含数据结构定义 */
#include "string.h"         /* 字符串操作函数（memcpy 等） */
#include "usart.h"          /* UART 外设句柄定义 */
#include "drv_uart.h"       /* UART 驱动层，包含 DMA 句柄和缓冲区长度定义 */

/* Exported variables --------------------------------------------------------*/
/**
 * @brief 全局 IMU 数据结构体
 * 通过 UART 接收并解析后的 IMU 数据存储在此全局变量中，
 * 其他模块可直接读取该变量获取 IMU 数据
 */
ImuUartData imu_data;

/* Private variables ---------------------------------------------------------*/
/**
 * @brief 数据结构体大小检查变量
 * 用于在运行时验证 ImuUartData 结构体大小是否与预期一致
 */
volatile uint16_t size_check;

/* Private constants ---------------------------------------------------------*/
/** @brief IMU 一帧数据的字节长度，包含帧头、数据和校验 */
#define IMU_FRAME_SIZE  64

/** @brief IMU 数据帧帧头标识，固定为 0xA5 */
#define IMU_FRAME_HEAD  0xA5

/** @brief 弧度转角度的转换系数 (180 / pi = 57.29578) */
#define RAD_TO_DEG      57.29578f

/**
 * @brief CRC8 查表法使用的查找表
 * 多项式: 0x31
 * 初始值: 0xFF
 * 该表用于加速 CRC8 校验计算，避免逐位计算的开销
 */
static const uint8_t CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/**
 * @brief  CRC8 校验计算 (多项式 0x31, 初始值 0xFF)
 * @param  data  待校验数据指针
 * @param  len   数据长度（字节数）
 * @return CRC8 校验值（8位无符号整数）
 *
 * @note   使用查表法实现，效率高于逐位计算
 *         校验算法：初始 CRC = 0xFF，每次取 CRC 高4位与数据异或查表
 */
uint8_t CRC8_Calc(uint8_t *data, uint16_t len) {
    uint8_t crc = 0xff;  /* CRC 初始值 0xFF */
    for (uint16_t i = 0; i < len; i++) {
        /* 当前 CRC 与数据字节异或，取低8位作为查表索引 */
        crc = CRC8_TAB[crc ^ data[i]];
    }
    return crc;
}

/**
 * @brief  IMU UART 初始化，清空数据缓存
 * @note   在 DEVICE_Init() 中被调用，完成 IMU UART 模块的初始化
 *         主要工作：清零 imu_data 结构体，确保初始状态干净
 */
void IMU_UART_Init(void)
{
    size_check = sizeof(ImuUartData);  /* 记录结构体大小，用于后续校验 */
    memset(&imu_data, 0, sizeof(ImuUartData));  /* 清零全局数据结构 */
}

/* USART1 数据接收处理 --------------------------------------------------------*/

/**
 * @brief  USART1 空闲中断数据处理器
 * @note   在 USART1 空闲中断中被 DRV_UART_IRQHandler() 调用
 *         负责从 DMA 接收缓冲区中解析 IMU 数据帧
 *         解析流程：查找帧头 -> 校验 CRC -> 存储数据 -> 弧度转角度
 *
 * @param  rxBuf  DMA 接收缓冲区指针
 *
 * @note   该函数在中断上下文中执行，需保证执行效率
 *         数据帧格式：[帧头0xA5][63字节数据+CRC8]
 */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
    /* 计算实际接收到的数据长度 = 缓冲区总长 - DMA剩余计数 */
    uint16_t rx_len = USART1_RX_BUF_LEN - hdma_usart1_rx.Instance->NDTR;
    uint16_t i = 0;

    while (i < rx_len)
    {
        /* 步骤1：在缓冲区中查找帧头 0xA5 */
        while (i < rx_len && rxBuf[i] != IMU_FRAME_HEAD)
        {
            i++;
        }

        /* 如果没有找到帧头，或者剩余数据不足以组成一帧，则退出 */
        if (i >= rx_len)
        {
            break;
        }
        if (i + IMU_FRAME_SIZE > rx_len)
        {
            break;
        }

        /* 步骤2：将一帧数据拷贝到临时缓冲区进行处理 */
        uint8_t parse_buf[IMU_FRAME_SIZE];
        memcpy(parse_buf, &rxBuf[i], IMU_FRAME_SIZE);

        /* 步骤3：CRC8 校验 - 计算前63字节的CRC，与最后一字节比较 */
        uint8_t calc_crc = CRC8_Calc(parse_buf, IMU_FRAME_SIZE - 1);
        uint8_t recv_crc = parse_buf[IMU_FRAME_SIZE - 1];

        /* 步骤4：校验通过则解析数据到全局结构体 */
        if (calc_crc == recv_crc)
        {
            /* 将整帧数据拷贝到全局数据结构体 */
            memcpy(&imu_data, parse_buf, sizeof(ImuUartData));

            /* 欧拉角弧度转度：IMU内部输出为弧度，转换为角度制方便后续使用 */
            imu_data.eulr_.yaw *= RAD_TO_DEG;
            imu_data.eulr_.pit *= RAD_TO_DEG;
            imu_data.eulr_.rol *= RAD_TO_DEG;
        }

        /* 跳过当前帧，继续处理下一帧（支持一帧中包含多个数据帧） */
        i += IMU_FRAME_SIZE;
    }
}
