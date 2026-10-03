/**
 ******************************************************************************
 * @file    drv_can.c
 * @brief   CAN（Controller Area Network）底层驱动文件
 ******************************************************************************
 * @attention
 *
 * 该文件实现了FDCAN（Flexible Data-rate CAN）总线的底层驱动功能，包括：
 * - CAN外设的初始化与滤波器配置
 * - CAN数据的发送与接收
 * - 接收中断回调处理
 * - CAN总线的重启功能
 *
 * 支持三个FDCAN实例：FDCAN1、FDCAN2、FDCAN3
 * CAN总线在机器人系统中用于与电机、传感器等设备进行通信
 *
 * Copyright 2024 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
/* 包含CAN驱动头文件和FDCAN HAL库头文件 */
#include "drv_can.h"
#include "fdcan.h"

/* Exported variables --------------------------------------------------------*/
/* 导出变量（当前文件内未使用） */

/* Private function prototypes -----------------------------------------------*/
/* 私有函数前置声明：三个CAN通道的接收数据处理函数（弱定义，在协议层实现） */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN3_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

/* Private variables ---------------------------------------------------------*/
/**
 * @brief FDCAN3实例的外部句柄声明
 */
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief 三个CAN通道的接收帧结构体
 *        用于存储从CAN总线接收到的数据帧信息和数据内容
 */
CAN_RxFrameTypeDef hcan1RxFrame;   /* CAN1接收帧 */
CAN_RxFrameTypeDef hcan2RxFrame;   /* CAN2接收帧 */
CAN_RxFrameTypeDef hcan3RxFrame;   /* CAN3接收帧 */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  FDCAN1重启函数
 * @param  无
 * @retval 无
 * @note   当CAN1通信出现异常时，可以调用此函数重启CAN1
 *         重启流程：反初始化 -> 重新初始化 -> 重新配置滤波器 -> 启动 -> 使能中断
 */
void FDCAN1_Restart(void) {
    /* 步骤1：反初始化FDCAN1模块，停止当前所有操作 */
    HAL_FDCAN_DeInit(&hfdcan1);

    /* 步骤2：重新初始化FDCAN1，应用CubeMX生成的配置参数 */
    MX_FDCAN1_Init();

    /* 步骤3：重新配置CAN1接收滤波器 */
    CAN1_Filter_Init();

    /* 步骤4：启动FDCAN1模块 */
    HAL_FDCAN_Start(&hfdcan1);

    /* 步骤5：使能FIFO0新消息中断和错误警告中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING, 0);
}

/**
 * @brief  FDCAN2重启函数
 * @param  无
 * @retval 无
 * @note   当CAN2通信出现异常时，可以调用此函数重启CAN2
 *         重启流程与FDCAN1相同
 */
void FDCAN2_Restart(void) {
    /* 步骤1：反初始化FDCAN2模块 */
    HAL_FDCAN_DeInit(&hfdcan2);

    /* 步骤2：重新初始化FDCAN2 */
    MX_FDCAN2_Init();

    /* 步骤3：重新配置CAN2接收滤波器 */
    CAN2_Filter_Init();

    /* 步骤4：启动FDCAN2模块 */
    HAL_FDCAN_Start(&hfdcan2);

    /* 步骤5：使能FIFO0新消息中断和错误警告中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING, 0);
}

/**
 * @brief  FDCAN FIFO0接收中断回调函数
 * @param  hcan: FDCAN句柄指针，标识是哪个CAN实例触发的中断
 * @param  RxFifo0ITs: FIFO0中断状态标志
 * @retval 无
 * @note   该函数在stm32g4xx_hal_fdcan.c中弱定义，此处重写
 *         当FIFO0收到新消息时由HAL库自动调用
 *         根据CAN实例句柄判断数据来源，分别调用对应的处理函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
  /* 判断是否为FDCAN1接收的数据 */
  if(hcan == &hfdcan1)
  {
    /* 从FIFO0中读取接收到的数据帧，存入hcan1RxFrame */
    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
    /* 调用CAN1数据处理函数（协议层实现） */
    CAN1_rxDataHandler(hcan1RxFrame.header.Identifier, hcan1RxFrame.data);
  }
  /* 判断是否为FDCAN2接收的数据 */
  else if(hcan == &hfdcan2)
  {
    /* 从FIFO0中读取接收到的数据帧，存入hcan2RxFrame */
    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
    /* 调用CAN2数据处理函数（协议层实现） */
    CAN2_rxDataHandler(hcan2RxFrame.header.Identifier, hcan2RxFrame.data);
  }
  /* 其他CAN实例不处理，直接返回 */
  else
  {
    return;
  }
}

/**
 * @brief  FDCAN3接收帧头部结构体（用于FIFO1）
 */
FDCAN_RxHeaderTypeDef RxHeader2;

/**
 * @brief  FDCAN FIFO1接收中断回调函数
 * @param  hfdcan: FDCAN句柄指针
 * @param  RxFifo1ITs: FIFO1中断状态标志
 * @retval 无
 * @note   FDCAN3使用FIFO1进行数据接收，与FDCAN1/2使用FIFO0不同
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  /* 检查FIFO1是否有新消息中断标志 */
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    /* 判断是否为FDCAN3实例 */
    if(hfdcan->Instance == FDCAN3)
    {
      /* 从FIFO1中读取接收到的数据帧 */
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &hcan3RxFrame.header, hcan3RxFrame.data);
      /* 调用CAN3数据处理函数（协议层实现） */
      CAN3_rxDataHandler(hcan3RxFrame.header.Identifier, hcan3RxFrame.data);
    }
  }
}


/**
 * @brief  CAN数据发送函数
 * @param  hcan:  FDCAN句柄指针，指定使用哪个CAN实例发送
 * @param  stdId: 标准CAN ID（11位），标识发送数据的目标设备
 * @param  dat:   8字节数据数组指针，要发送的数据内容
 * @retval HAL_StatusTypeDef: 返回HAL状态，通常为HAL_OK
 * @note   该函数配置标准CAN数据帧格式，发送8字节经典CAN帧
 *         如果检测到错误计数器不为0，则清除INIT位以恢复通信
 */
HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)
{
  /* 定义发送消息头结构体 */
  FDCAN_TxHeaderTypeDef tx_message;

  /* 配置发送帧参数 */
  tx_message.IdType = FDCAN_STANDARD_ID;          /* 使用标准ID（11位） */
  tx_message.Identifier = stdId;                    /* 设置CAN标识符 */
  tx_message.DataLength = FDCAN_DLC_BYTES_8;       /* 数据长度为8字节 */
  tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 错误状态指示：主动 */
  tx_message.BitRateSwitch = FDCAN_BRS_OFF;        /* 不使用比特率切换（经典CAN模式） */
  tx_message.FDFormat = FDCAN_CLASSIC_CAN;          /* 经典CAN格式，非CANFD */
  tx_message.TxFrameType = FDCAN_DATA_FRAME;        /* 数据帧类型（非远程帧） */
  tx_message.MessageMarker = 0;                     /* 消息标记（用于TX事件FIFO） */
  tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /* 不使用TX事件FIFO */

  /* 检查错误计数器，如果存在错误则清除INIT位恢复通信 */
  if (hcan->Instance->ECR != 0U)
  {
    CLEAR_BIT(hcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }

  /* 将数据帧添加到发送FIFO队列中 */
  HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_message, dat);

  return HAL_OK;
}

/* rxData Handler [Weak] functions -------------------------------------------*/

/**
 * @brief  [__WEAK] CAN1接收数据处理函数（弱定义）
 * @param  rxId: 接收到的CAN报文ID
 * @param  rxBuf: 接收到的8字节数据缓冲区指针
 * @note   该函数为弱定义，需要在协议层(Protocol Layer)中重写
 *         实现具体的CAN1数据解析和处理逻辑
 *         例如：解析电机反馈数据、传感器数据等
 */
__WEAK void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
 * @brief  [__WEAK] CAN2接收数据处理函数（弱定义）
 * @param  rxId: 接收到的CAN报文ID
 * @param  rxBuf: 接收到的8字节数据缓冲区指针
 * @note   该函数为弱定义，需要在协议层(Protocol Layer)中重写
 *         实现具体的CAN2数据解析和处理逻辑
 */
__WEAK void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
 * @brief  [__WEAK] CAN3接收数据处理函数（弱定义）
 * @param  rxId: 接收到的CAN报文ID
 * @param  rxBuf: 接收到的8字节数据缓冲区指针
 * @note   该函数为弱定义，需要在协议层(Protocol Layer)中重写
 *         实现具体的CAN3数据解析和处理逻辑
 */
__WEAK void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
 * @brief  CAN1滤波器初始化函数
 * @param  无
 * @retval 无
 * @note   配置FDCAN1的接收滤波器和全局过滤规则：
 *         - 使用标准ID过滤器（11位ID）
 *         - 过滤器0：掩码模式，ID1=0x000，ID2=0x000，接收所有标准ID
 *         - 全局过滤：不匹配的标准ID和扩展ID均拒绝，过滤远程帧
 *         - 使能FIFO0新消息中断
 *         - 启动FDCAN1
 */
void CAN1_Filter_Init(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* 配置接收滤波器 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;           /* 使用标准ID过滤 */
  sFilterConfig.FilterIndex = 0;                       /* 使用滤波器索引0 */
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;        /* 掩码过滤模式 */
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /* 匹配的数据存入FIFO0 */
  sFilterConfig.FilterID1 = 0x00000000;                /* 过滤器ID1：0x000（不屏蔽任何位） */
  sFilterConfig.FilterID2 = 0x00000000;                /* 过滤器掩码ID2：0x000（接受所有ID） */
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);    /* 应用滤波器配置 */

  /* 全局过滤设置：拒绝不匹配的标准ID和扩展ID，过滤远程帧 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
    FDCAN_REJECT,           /* 不匹配的标准ID帧：拒绝 */
    FDCAN_REJECT,           /* 不匹配的扩展ID帧：拒绝 */
    FDCAN_FILTER_REMOTE,    /* 标准ID远程帧：过滤（拒绝） */
    FDCAN_FILTER_REMOTE);   /* 扩展ID远程帧：过滤（拒绝） */

  /* 使能FIFO0新消息中断 */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  /* 启动FDCAN1 */
  HAL_FDCAN_Start(&hfdcan1);
}

/**
 * @brief  CAN2滤波器初始化函数
 * @param  无
 * @retval 无
 * @note   配置与CAN1相同的滤波器规则，应用于FDCAN2实例
 */
void CAN2_Filter_Init(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* 配置接收滤波器 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;              /* 标准ID过滤 */
  sFilterConfig.FilterIndex = 0;                          /* 滤波器索引0 */
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;           /* 掩码过滤模式 */
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  /* 存入FIFO0 */
  sFilterConfig.FilterID1 = 0x00000000;                   /* 接受所有ID */
  sFilterConfig.FilterID2 = 0x00000000;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

  /* 全局过滤设置 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
    FDCAN_REJECT,
    FDCAN_REJECT,
    FDCAN_FILTER_REMOTE,
    FDCAN_FILTER_REMOTE);

  /* 使能FIFO0新消息中断 */
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  /* 启动FDCAN2 */
  HAL_FDCAN_Start(&hfdcan2);
}

/**
 * @brief  CAN3滤波器初始化函数
 * @param  无
 * @retval 无
 * @note   CAN3使用FIFO1接收数据（与CAN1/CAN2使用FIFO0不同）
 *         滤波器索引为1，避免与FDCAN1/2的滤波器冲突
 */
void CAN3_Filter_Init(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* 配置接收滤波器 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;              /* 标准ID过滤 */
  sFilterConfig.FilterIndex = 1;                          /* 滤波器索引1（与FDCAN1/2区分） */
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;           /* 掩码过滤模式 */
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;  /* 存入FIFO1（不是FIFO0） */
  sFilterConfig.FilterID1 = 0x00000000;                   /* 接受所有ID */
  sFilterConfig.FilterID2 = 0x00000000;
  HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);

  /* 全局过滤设置 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
    FDCAN_REJECT,
    FDCAN_REJECT,
    FDCAN_FILTER_REMOTE,
    FDCAN_FILTER_REMOTE);

  /* 使能FIFO1新消息中断（注意是FIFO1，不是FIFO0） */
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  /* 启动FDCAN3 */
  HAL_FDCAN_Start(&hfdcan3);
}
