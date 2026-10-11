/**
 ******************************************************************************
 * @file        drv_uart.c
 * @author      RobotPilots@2020
 * @brief       UART驱动包 - 基于HAL库的串口通信底层驱动
 ******************************************************************************
 * @attention
 *
 * 该文件实现了STM32多个UART/USART串口的底层驱动，包括：
 * - 串口初始化（DMA接收模式）
 * - 空闲中断（IDLE）检测与处理
 * - DMA双缓冲接收机制
 * - 串口数据接收回调处理
 * - 格式化打印输出功能
 *
 * 支持的串口实例：
 *   - USART1  : 用于特定外设通信
 *   - UART5   : 用于遥控器（DBUS）通信
 *   - UART7   : 用于调试输出或扩展通信
 *   - UART8   : 用于扩展通信
 *   - UART9   : 用于扩展通信
 *   - USART10 : 用于特定外设通信
 *
 * 接收机制说明：
 *   - 使用DMA + IDLE中断实现高效数据接收
 *   - IDLE中断：当串口总线空闲（一个字节时间无新数据）时触发
 *   - DMA双缓冲：UART5支持DMA双缓冲模式，避免数据覆盖
 *
 * Copyright 2020 RobotPilots
 *
 * @Version     V1.0
 * @date        15-August-2020
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"
#include "string.h"
#include "judge_protocol.h"   /* 裁判系统协议头文件 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 外部串口句柄声明（在CubeMX生成的代码中定义） */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart10;

/* Private macro -------------------------------------------------------------*/

/**
 * @brief UART5（遥控器DBUS）接收数据帧长度定义
 *        遥控器DBUS协议一帧数据为18字节
 */
#define USART5_RX_DATA_FRAME_LEN  (18u)

/**
 * @brief UART5接收缓冲区长度
 *        与数据帧长度相同，确保能完整接收一帧数据
 */
#define USART5_RX_BUF_LEN         (USART5_RX_DATA_FRAME_LEN)

/**
 * @brief 通用接收缓冲区大小
 *        用于DMA接收的临时缓冲区
 */
#define BUFF_SIZE 512

/**
 * @brief DMA接收临时缓冲区
 */
uint8_t rx_buff[BUFF_SIZE];

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 各串口接收数据处理函数（弱定义）
 *        需要在协议层(Protocol Layer)中重写实现具体的协议解析
 */
__WEAK void USART10_rxDataHandler(uint8_t *rxBuf);  /* 串口10数据处理 */
__WEAK void USART9_rxDataHandler(uint8_t *rxBuf);   /* 串口9数据处理 */
__WEAK void USART8_rxDataHandler(uint8_t *rxBuf);   /* 串口8数据处理 */
__WEAK void USART7_rxDataHandler(uint8_t *rxBuf);   /* 串口7数据处理 */
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf);   /* 串口5数据处理（遥控器） */
__WEAK void USART1_rxDataHandler(const uint8_t *rxBuf, uint16_t length);   /* 串口1数据处理 */

/**
 * @brief DMA启动传输函数（内部使用）
 */
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma,
                            uint32_t SrcAddress,
                            uint32_t DstAddress,
                            uint32_t DataLength);

/**
 * @brief DMA双缓冲模式下，内存0接收完成回调
 */
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);

/**
 * @brief DMA双缓冲模式下，内存1接收完成回调
 */
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);

/**
 * @brief 串口空闲中断回调函数
 *        当检测到串口空闲时，处理已接收的数据
 */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);

/**
 * @brief UART5遥控器启动接收函数
 */
static void uart5_rc_start_receive(void);

/**
 * @brief DMA多缓冲区启动传输函数（不使用中断）
 */
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma,
                                                    uint32_t SrcAddress,
                                                    uint32_t DstAddress,
                                                    uint32_t SecondMemAddress,
                                                    uint32_t DataLength);

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 各串口DMA接收缓冲区
 *        使用__attribute__((section (".AXI_SRAM")))将缓冲区放置在AXI SRAM中
 *        AXI SRAM带宽更高，适合DMA高速传输
 */
__attribute__((section (".AXI_SRAM"))) uint8_t usart1_dma_rxbuf[USART1_RX_BUF_LEN];    /* 串口1 DMA接收缓冲区 */
__attribute__((section (".AXI_SRAM"))) uint8_t usart10_dma_rxbuf[USART10_RX_BUF_LEN];  /* 串口10 DMA接收缓冲区 */
__attribute__((section (".AXI_SRAM"))) uint8_t usart7_dma_rxbuf[USART7_RX_BUF_LEN];    /* 串口7 DMA接收缓冲区 */
__attribute__((section (".AXI_SRAM"))) uint8_t usart8_dma_rxbuf[USART8_RX_BUF_LEN];    /* 串口8 DMA接收缓冲区 */
__attribute__((section (".AXI_SRAM"))) uint8_t usart9_dma_rxbuf[USART9_RX_BUF_LEN];    /* 串口9 DMA接收缓冲区 */

/**
 * @brief UART5（遥控器）DMA接收缓冲区
 *        根据配置选择单缓冲或双缓冲模式
 */
#if UART5_RC_USE_HAL_TOIDLE_DMA
/* 使用HAL TOIDLE DMA模式：单缓冲区 */
__attribute__((section (".AXI_SRAM"))) uint8_t usart5_dma_rxbuf[USART5_RX_BUF_LEN];
#else
/* 使用双缓冲模式：两个缓冲区交替使用，避免数据覆盖 */
__attribute__((section (".AXI_SRAM"))) uint8_t usart5_dma_rxbuf[2][USART5_RX_BUF_LEN];
#endif

/* Exported variables --------------------------------------------------------*/
/* 导出变量（当前未使用） */


/**
 * @brief  HAL库UART接收事件回调函数（重写）
 * @param  huart: 触发事件的UART句柄指针
 * @param  Size:  实际接收到的数据字节数
 * @retval 无
 * @note   当使用HAL_UARTEx_ReceiveToIdle_DMA()方式接收时，
 *         数据接收完成（TC）或空闲中断（IDLE）都会触发此回调
 *         根据UART实例分别处理：
 *         - UART5：遥控器DBUS数据处理
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  /* ====== UART5 遥控器数据处理 ====== */
  if(huart->Instance == UART5)
  {
#if UART5_RC_USE_HAL_TOIDLE_DMA
    /*
     * 使用HAL TOIDLE DMA模式的处理逻辑：
     * DBUS帧长度和DMA缓冲区长度都是18字节。
     * 完整的一帧可能以HAL_UART_RXEVENT_TC（传输完成）而非IDLE触发回调。
     * 无论事件类型是IDLE、TC还是意外的短帧，
     * 都要解析完整的帧数据并重启接收。
     */
    if(Size == USART5_RX_DATA_FRAME_LEN)
    {
      /* 数据长度匹配，调用遥控器数据处理函数 */
      USART5_rxDataHandler(usart5_dma_rxbuf);
    }
    /* 无论是否成功解析，都重启接收以准备接收下一帧 */
    uart5_rc_start_receive();
#else
    /*
     * 使用DMA双缓冲模式的处理逻辑：
     * 通过检查DMA的CT(Current Target)位判断当前使用哪个缓冲区
     */
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
      /* CT=0：当前使用内存0（M0AR），接收的数据在缓冲区[0]中 */
      __HAL_DMA_DISABLE(huart->hdmarx);       /* 先禁用DMA */

      /* 切换到内存1作为下一个接收目标 */
      ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

      /* 设置DMA接收长度为双倍（两个缓冲区总长度） */
      __HAL_DMA_SET_COUNTER(huart->hdmarx, USART5_RX_DATA_FRAME_LEN * 2);

      /* 如果接收到完整的一帧，解析缓冲区[0]的数据 */
      if(Size == USART5_RX_DATA_FRAME_LEN)
      {
        USART5_rxDataHandler(usart5_dma_rxbuf[0]);
      }
    }
    else
    {
      /* CT=1：当前使用内存1（M1AR），接收的数据在缓冲区[1]中 */
      __HAL_DMA_DISABLE(huart->hdmarx);       /* 先禁用DMA */

      /* 切换回内存0作为下一个接收目标 */
      ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

      /* 设置DMA接收长度为双倍 */
      __HAL_DMA_SET_COUNTER(huart->hdmarx, USART5_RX_DATA_FRAME_LEN * 2);

      /* 如果接收到完整的一帧，解析缓冲区[1]的数据 */
      if(Size == USART5_RX_DATA_FRAME_LEN)
      {
        USART5_rxDataHandler(usart5_dma_rxbuf[1]);
      }
    }
#endif
  }

  /* 非HAL TOIDLE模式下，需要手动重启DMA接收 */
#if !UART5_RC_USE_HAL_TOIDLE_DMA
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;  /* 设置接收类型为IDLE检测 */

  /* 使能IDLE空闲中断 */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* 使能串口DMA接收请求 */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* 使能DMA传输 */
  __HAL_DMA_ENABLE(huart->hdmarx);
#endif
}

/**
 * @brief  HAL库UART错误回调函数（重写）
 * @param  huart: 发生错误的UART句柄指针
 * @retval 无
 * @note   USART1 错误后中止接收并重启 DMA；UART5 保留 TOIDLE 恢复路径。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    /* 错误帧不提交；恢复 DMA，后续裁判帧由 CRC 重新同步。 */
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_FE);
    /* DME 可能已把 UART 置 READY，但 DMA 仍 BUSY；按 DMAR 中止接收。 */
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    HAL_UART_Receive_DMA(huart, usart1_dma_rxbuf, USART1_RX_BUF_LEN);
  }
#if UART5_RC_USE_HAL_TOIDLE_DMA
  else if(huart->Instance == UART5)
  {
    /* 发生错误后重启UART5接收 */
    uart5_rc_start_receive();
  }
#endif
}

/**
 * @brief  UART DMA双缓冲区初始化函数（内部使用）
 * @param  huart:            UART句柄指针
 * @param  DstAddress:       内存0的目标地址（第一个缓冲区）
 * @param  SecondMemAddress: 内存1的目标地址（第二个缓冲区）
 * @param  DataLength:       每个缓冲区的数据长度
 * @retval 无
 * @note   配置DMA双缓冲传输模式：
 *         1. 设置接收类型为IDLE检测
 *         2. 使能UART的DMA接收功能
 *         3. 使能IDLE中断
 *         4. 配置DMA的源地址、目标地址和传输长度
 *         5. 使能DMA双缓冲模式（DBM位）
 *         6. 启动DMA传输
 */
static void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
  /* 设置接收类型为IDLE空闲检测 */
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* 设置接收总大小（双缓冲：数据长度 * 2） */
  huart->RxXferSize = DataLength * 2;

  /* 使能串口DMA接收功能 */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* 使能UART空闲中断（IDLE） */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* 关闭DMA传输，确保可以安全配置DMA参数 */
  do{
    __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);  /* 等待DMA完全关闭 */

  /* 配置DMA外设地址（源地址）：串口数据寄存器 */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* 配置DMA内存0地址：第一个缓冲区 */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* 配置DMA内存1地址：第二个缓冲区 */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* 配置DMA数据传输长度 */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* 使能DMA双缓冲模式（DBM位） */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* 使能DMA传输 */
  __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
 * @brief  UART5（遥控器DBUS）初始化函数
 * @param  无
 * @retval 无
 * @note   根据配置选择初始化方式：
 *         - HAL TOIDLE DMA模式：使用HAL库的接收方式
 *         - 双缓冲模式：直接配置DMA双缓冲区
 */
void USART5_Init(void)
{
#if UART5_RC_USE_HAL_TOIDLE_DMA
  /* 使用HAL TOIDLE DMA模式启动接收 */
  uart5_rc_start_receive();
#else
  /* 使用DMA双缓冲模式初始化 */
  USART_DMAEx_MultiBuffer_Init(&huart5, (uint32_t*)&usart5_dma_rxbuf[0],
                                (uint32_t*)&usart5_dma_rxbuf[1], USART5_RX_BUF_LEN);
#endif
}

/**
 * @brief  USART1初始化函数
 * @param  无
 * @retval 无
 * @note   使能IDLE中断，启动DMA接收
 */
/* Normal-mode USART1 DMA completion handles chunks with no idle gap. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    USART1_rxDataHandler(usart1_dma_rxbuf, USART1_RX_BUF_LEN);
    HAL_UART_Receive_DMA(huart, usart1_dma_rxbuf, USART1_RX_BUF_LEN);
  }
}

void USART1_Init(void)
{
  /* 使能UART空闲中断 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  /* 启动DMA接收，将数据存入usart1_dma_rxbuf */
  HAL_UART_Receive_DMA(&huart1, usart1_dma_rxbuf, USART1_RX_BUF_LEN);
}

/**
 * @brief  USART10初始化函数
 * @param  无
 * @retval 无
 * @note   使能IDLE中断，启动DMA接收
 */
void USART10_Init(void)
{
  /* 使能UART空闲中断 */
  __HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);

  /* 启动DMA接收 */
  HAL_UART_Receive_DMA(&huart10, usart10_dma_rxbuf, USART10_RX_BUF_LEN);
}

/**
 * @brief  UART7初始化函数
 * @param  无
 * @retval 无
 * @note   使能IDLE中断，启动DMA接收
 */
void USART7_Init(void)
{
  /* 使能UART空闲中断 */
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);

  /* 启动DMA接收 */
  HAL_UART_Receive_DMA(&huart7, usart7_dma_rxbuf, USART7_RX_BUF_LEN);
}

/**
 * @brief  UART8初始化函数
 * @param  无
 * @retval 无
 * @note   使能IDLE中断，启动DMA接收
 */
void USART8_Init(void)
{
  /* 使能UART空闲中断 */
  __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

  /* 启动DMA接收 */
  HAL_UART_Receive_DMA(&huart8, usart8_dma_rxbuf, USART8_RX_BUF_LEN);
}

/**
 * @brief  UART9初始化函数
 * @param  无
 * @retval 无
 * @note   使能IDLE中断，启动DMA接收
 */
void USART9_Init(void)
{
  /* 使能UART空闲中断 */
  __HAL_UART_ENABLE_IT(&huart9, UART_IT_IDLE);

  /* 启动DMA接收 */
  HAL_UART_Receive_DMA(&huart9, usart9_dma_rxbuf, USART9_RX_BUF_LEN);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  串口空闲中断回调函数
 * @param  huart: 触发空闲中断的UART句柄指针
 * @retval 无
 * @note   当串口接收完一帧数据后进入空闲状态时触发
 *         USART1：清除空闲标志 -> 停止DMA -> 解析实际长度 -> 重启DMA
 *         根据不同的UART实例分别调用对应的rxDataHandler
 */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
  /* 清除UART空闲中断标志位 */
  __HAL_UART_CLEAR_IDLEFLAG(huart);

  /* ====== USART10 处理 ====== */
  if (huart == &huart10)
  {
    /* 停止DMA传输 */
    __HAL_DMA_DISABLE(huart->hdmarx);
    /* 重新启动DMA接收 */
    HAL_UART_Receive_DMA(&huart10, usart10_dma_rxbuf, USART10_RX_BUF_LEN);
    /* 调用串口10数据处理函数 */
    USART10_rxDataHandler(usart10_dma_rxbuf);
    /* 重启DMA使能 */
    __HAL_DMA_ENABLE(huart->hdmarx);
  }

  /* ====== USART1 处理 ====== */
  else if (huart == &huart1)
  {
    uint16_t received;
    /* Clearing EN may generate TC: mask callbacks before stopping the stream.
     * USART1 DMA IRQ has higher priority than UART IDLE; never let TC restart
     * reception or reenter the parser while this short chunk is being handled. */
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_FE);
    /* Stop hardware before sampling NDTR; DMAStop also restores HAL RxState. */
    __HAL_DMA_DISABLE(huart->hdmarx);
    while (((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN) {}
    received = USART1_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    HAL_UART_DMAStop(huart);
    if (received <= USART1_RX_BUF_LEN)
      USART1_rxDataHandler(usart1_dma_rxbuf, received);
    HAL_UART_Receive_DMA(huart, usart1_dma_rxbuf, USART1_RX_BUF_LEN);
  }

  /* ====== UART7 处理 ====== */
  else if (huart == &huart7)
  {
    __HAL_DMA_DISABLE(huart->hdmarx);
    HAL_UART_Receive_DMA(&huart7, usart7_dma_rxbuf, USART7_RX_BUF_LEN);
    USART7_rxDataHandler(usart7_dma_rxbuf);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }

  /* ====== UART8 处理 ====== */
  else if (huart == &huart8)
  {
    __HAL_DMA_DISABLE(huart->hdmarx);
    HAL_UART_Receive_DMA(&huart8, usart8_dma_rxbuf, USART8_RX_BUF_LEN);
    USART8_rxDataHandler(usart8_dma_rxbuf);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }

  /* ====== UART9 处理 ====== */
  else if (huart == &huart9)
  {
    __HAL_DMA_DISABLE(huart->hdmarx);
    HAL_UART_Receive_DMA(&huart9, usart9_dma_rxbuf, USART9_RX_BUF_LEN);
    USART9_rxDataHandler(usart9_dma_rxbuf);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
}

/**
 * @brief  UART5遥控器启动接收函数（HAL TOIDLE DMA模式）
 * @param  无
 * @retval 无
 * @note   停止当前DMA传输，清除所有错误标志，重新启动IDLE DMA接收
 *         关闭半传输完成中断（HT），只在完整接收时触发回调
 */
static void uart5_rc_start_receive(void)
{
#if UART5_RC_USE_HAL_TOIDLE_DMA
  /* 停止UART5的DMA传输 */
  HAL_UART_DMAStop(&huart5);

  /* 清除所有UART错误标志（溢出、噪声、帧错误、空闲等） */
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);

  /* 启动IDLE DMA接收：接收到数据后在空闲时触发回调 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_rxbuf, USART5_RX_BUF_LEN);

  /* 关闭半传输完成中断（HT），仅保留完整传输完成中断 */
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
#endif
}


/**
 * @brief  DMA启动传输函数（内部使用，不使用中断）
 * @param  hdma:       DMA句柄指针
 * @param  SrcAddress: 源地址（外设地址或内存地址）
 * @param  DstAddress: 目标地址（内存地址或外设地址）
 * @param  DataLength: 传输数据长度
 * @retval HAL_StatusTypeDef: 传输状态
 * @note   手动配置DMA寄存器启动传输，适用于需要精细控制的场景
 */
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma,
                            uint32_t SrcAddress,
                            uint32_t DstAddress,
                            uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* 锁定DMA句柄，防止并发访问 */
  __HAL_LOCK(hdma);

  /* 检查DMA是否处于就绪状态 */
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /* 设置DMA状态为忙碌 */
    hdma->State = HAL_DMA_STATE_BUSY;

    /* 清除错误码 */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* 清除双缓冲模式位（DBM），使用单缓冲模式 */
    ((DMA_Stream_TypeDef  *)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);

    /* 配置DMA传输数据长度 */
    ((DMA_Stream_TypeDef  *)hdma->Instance)->NDTR = DataLength;

    /* 根据传输方向配置源地址和目标地址 */
    if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
    {
      /* 内存到外设方向：PAR=外设地址，M0AR=内存地址 */
      ((DMA_Stream_TypeDef  *)hdma->Instance)->PAR = DstAddress;
      ((DMA_Stream_TypeDef  *)hdma->Instance)->M0AR = SrcAddress;
    }
    else
    {
      /* 外设到内存方向：PAR=外设地址，M0AR=内存地址 */
      ((DMA_Stream_TypeDef  *)hdma->Instance)->PAR = SrcAddress;
      ((DMA_Stream_TypeDef  *)hdma->Instance)->M0AR = DstAddress;
    }

    /* 使能DMA传输 */
    __HAL_DMA_ENABLE(hdma);
  }
  else
  {
    /* DMA不在就绪状态，解锁并返回忙状态 */
    __HAL_UNLOCK(hdma);
    status = HAL_BUSY;
  }
  return status;
}

/**
 * @brief  串口中断处理函数（外部调用入口）
 * @param  huart: UART句柄指针
 * @retval 无
 * @note   该函数应在USARTx_IRQHandler()中断服务函数中被调用
 *         检测空闲中断标志并调用回调函数处理
 *         使用方法：在stm32f4xx_it.c的中断函数中调用 DRV_UART_IRQHandler(&huartx)
 */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  /* 检查是否为空闲中断标志，并且空闲中断使能 */
  if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
      __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
  {
    /* 调用空闲中断回调函数处理接收数据 */
    uart_rx_idle_callback(huart);
  }
}

/**
 * @brief  调试串口打印函数（格式化输出到UART7）
 * @param  format: 格式化字符串（与printf格式相同）
 * @param  ...: 可变参数列表
 * @retval 无
 * @note   将格式化后的字符串通过UART7发送
 *         缓冲区大小为100字节
 */
void WL_UART_printf(char *format, ...)
{
  char String[100];              /* 格式化字符串缓冲区 */
  va_list arg;                   /* 可变参数列表 */
  va_start(arg, format);         /* 初始化可变参数列表 */
  vsprintf(String, format, arg); /* 格式化字符串 */
  va_end(arg);                   /* 结束可变参数处理 */

  /* 通过UART7阻塞发送，超时1000ms */
  HAL_UART_Transmit(&huart7, (uint8_t *)String, strlen(String), 1000);
}

/* rxData Handler [Weak] functions -------------------------------------------*/

/**
 * @brief  [__WEAK] USART1接收数据处理函数（弱定义）
 * @param  rxBuf: 接收到的数据缓冲区指针
 * @note   需要在协议层(Protocol Layer)中重写
 *         实现USART1的具体数据解析逻辑
 */
__WEAK void USART1_rxDataHandler(const uint8_t *rxBuf, uint16_t length)
{
  (void)rxBuf;
  (void)length;
}

/**
 * @brief  [__WEAK] UART7接收数据处理函数（弱定义）
 * @param  rxBuf: 接收到的数据缓冲区指针
 * @note   需要在协议层(Protocol Layer)中重写
 */
__WEAK void USART7_rxDataHandler(uint8_t *rxBuf)
{
}

/**
 * @brief  [__WEAK] UART5接收数据处理函数（弱定义）
 * @param  rxBuf: 接收到的遥控器数据缓冲区指针
 * @note   需要在协议层(Protocol Layer)中重写
 *         实现DBUS遥控器协议解析
 */
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf)
{
}

/**
 * @brief  [__WEAK] USART10接收数据处理函数（弱定义）
 * @param  rxBuf: 接收到的数据缓冲区指针
 * @note   需要在协议层(Protocol Layer)中重写
 */
__WEAK void USART10_rxDataHandler(uint8_t *rxBuf)
{
}

/**
 * @brief  [__WEAK] UART8接收数据处理函数（弱定义）
 * @param  rxBuf: 接收到的数据缓冲区指针
 * @note   需要在协议层(Protocol Layer)中重写
 */
__WEAK void USART8_rxDataHandler(uint8_t *rxBuf)
{
}

/**
 * @brief  [__WEAK] UART9接收数据处理函数（弱定义）
 * @param  rxBuf: 接收到的数据缓冲区指针
 * @note   需要在协议层(Protocol Layer)中重写
 */
__WEAK void USART9_rxDataHandler(uint8_t *rxBuf)
{
}
