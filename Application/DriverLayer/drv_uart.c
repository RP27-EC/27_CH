/**
 ******************************************************************************
 * @file        drv_uart.c
 * @author      RobotPilots@2020
 * @brief       串口底层驱动包(基于HAL库) - 提供多路串口的DMA接收和空闲中断处理
 *              本文件实现了基于STM32 HAL库的串口驱动，核心特性包括：
 *              1. DMA双缓冲接收：USART3使用DMA双缓冲模式，避免数据覆盖丢失
 *              2. 空闲中断(IDLE)：一帧数据接收完毕后立即触发中断，降低延迟
 *              3. 多路串口支持：USART1(调试/IMU)、USART3(遥控器DBUS)、USART6(裁判系统)
 *              4. 弱定义回调函数：供协议层重写，实现具体的数据解析逻辑
 ******************************************************************************
 * @attention
 * Copyright 2020 RobotPilots
 *
 * @Version     V1.0
 * @date        15-August-2020
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"
#include "string.h"
#include <stdio.h>    /* 标准输入输出，用于串口打印函数 */
#include <stdarg.h>    /* 可变参数处理，用于printf类函数 */

/* 外部串口句柄（在CubeMX中生成） */
extern UART_HandleTypeDef huart1;   /* 串口1：通常用于调试或IMU通信 */
//extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart3;   /* 串口3：通常用于遥控器(DBUS)数据接收 */
//extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;   /* 串口6：通常用于裁判系统或其他外设通信 */

/* Private macro -------------------------------------------------------------*/
/**
  * USART3 接收协议帧长度宏定义
  * DBUS遥控器每帧数据固定为18字节
  */
#define USART3_RX_DATA_FRAME_LEN	(18u)	/* 遥控器数据帧长度: 18字节 */

/**
  * USART3 DMA接收缓冲区总长度
  * 比数据帧长度多6字节，提供冗余空间防止数据溢出
  */
#define USART3_RX_BUF_LEN			(USART3_RX_DATA_FRAME_LEN + 6u)	/* DMA缓冲区: 24字节 */

/**
  * USART6 DMA接收缓冲区长度
  * 用于裁判系统数据接收
  */
#define USART6_RX_BUF_LEN	  100	/* USART6接收缓冲区: 100字节 */

/* Private function prototypes -----------------------------------------------*/
/**
  * 串口数据处理回调函数声明（弱定义）
  * 在协议层(Protocol Layer)中重写以实现具体的数据解析
  */
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART3_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART6_rxDataHandler(uint8_t *rxBuf);

/* DMA双缓冲回调函数和空闲中断回调函数的前向声明 */
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
  * 各串口的DMA接收缓冲区
  * usart1_dma_rxbuf:     USART1单缓冲区
  * usart3_dma_rxbuf[2]:  USART3双缓冲区（[0]=Memory0, [1]=Memory1）
  * usart6_dma_rxbuf:     USART6单缓冲区
  */
uint8_t usart1_dma_rxbuf[USART1_RX_BUF_LEN];
uint8_t usart3_dma_rxbuf[2][USART3_RX_BUF_LEN];  /* 双缓冲: 第二维[0]和[1]交替使用 */
uint8_t usart6_dma_rxbuf[USART6_RX_BUF_LEN];

/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  DMA双缓冲Memory0接收完成回调函数
  *         当DMA将数据搬运到Memory0缓冲区完成后自动调用
  * @param  hdma: DMA句柄指针
  * @retval 无
  * @note   处理流程：
  *         1. 将DMA目标内存切换到Memory1（设置CT位）
  *         2. 处理Memory0中的数据
  *         3. 清空Memory0缓冲区
  */
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	/* 切换DMA目标内存为Memory1（CT=1表示当前使用Memory1） */
	hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
	/* 处理Memory0缓冲区中接收到的数据 */
	USART3_rxDataHandler(usart3_dma_rxbuf[0]);
	/* 清空Memory0缓冲区，为下一次接收做准备 */
	memset(usart3_dma_rxbuf[0], 0, USART3_RX_BUF_LEN);
}

/**
  * @brief  DMA双缓冲Memory1接收完成回调函数
  *         当DMA将数据搬运到Memory1缓冲区完成后自动调用
  * @param  hdma: DMA句柄指针
  * @retval 无
  * @note   处理流程：
  *         1. 将DMA目标内存切换到Memory0（清除CT位）
  *         2. 处理Memory1中的数据
  *         3. 清空Memory1缓冲区
  */
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	/* 切换DMA目标内存为Memory0（CT=0表示当前使用Memory0） */
	hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
	/* 处理Memory1缓冲区中接收到的数据 */
	USART3_rxDataHandler(usart3_dma_rxbuf[1]);
	/* 清空Memory1缓冲区，为下一次接收做准备 */
	memset(usart3_dma_rxbuf[1], 0, USART3_RX_BUF_LEN);
}

/**
  * @brief  串口接收空闲中断回调函数
  *         当串口接收完一帧数据后（总线空闲），由中断服务函数调用
  * @param  huart: 触发空闲中断的串口句柄指针
  * @retval 无
  * @note   空闲中断(IDLE Interrupt)的工作原理：
  *         当UART接收到一个字节后，如果在1个字符时间内没有新的数据，
  *         硬件会置位IDLE标志，触发中断。这表明一帧数据已接收完毕。
  *
  *         本函数处理流程：
  *         1. 清除空闲中断标志位
  *         2. 判断是哪个串口触发的中断
  *         3. 禁用DMA -> 处理数据 -> 清空缓冲区 -> 重启DMA
  *
  *         对于USART3(遥控器)：使用双缓冲模式
  *         对于USART1和USART6：使用普通DMA模式
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* 清除空闲中断标志位，避免重复触发中断 */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* ---------- USART3 处理（遥控器DBUS数据） ---------- */
	if (huart == &huart3)
	{
		/* 暂时禁用DMA接收，准备处理数据 */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/*
		 * 判断是否接收到完整的遥控器数据帧
		 * NDTR: DMA剩余传输字节数
		 * 已接收字节数 = 缓冲区总长度 - NDTR
		 * 如果已接收字节数 == 数据帧长度(18字节)，则接收到完整帧
		 */
		if ((USART3_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART3_RX_DATA_FRAME_LEN)
		{
			/*
			 * 通过DMA_SxCR_CT位判断当前使用的是哪个缓冲区：
			 * CT=1: 当前正在向Memory1写入，说明Memory0已接收完毕 -> 调用Memory0回调
			 * CT=0: 当前正在向Memory0写入，说明Memory1已接收完毕 -> 调用Memory1回调
			 */
			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);  /* Memory1接收完成 */
			else
				huart->hdmarx->XferCpltCallback(huart->hdmarx);    /* Memory0接收完成 */
		}

		/* 重新设置DMA传输计数器（缓冲区总长度） */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART3_RX_BUF_LEN);
		/* 重新使能DMA接收 */
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
	/* ---------- USART1 处理（调试/IMU数据） ---------- */
	else if (huart == &huart1)
	{
		/* 禁用DMA接收 */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* 处理USART1接收到的数据 */
		USART1_rxDataHandler(usart1_dma_rxbuf);
		/* 清空接收缓冲区 */
		memset(usart1_dma_rxbuf, 0, USART1_RX_BUF_LEN);
		/* 重置DMA传输计数器 */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART1_RX_BUF_LEN);

		/* 重新使能DMA接收 */
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
	/* ---------- USART6 处理（裁判系统数据） ---------- */
	else if (huart == &huart6)
	{
		/* 禁用DMA接收 */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* 处理USART6接收到的数据 */
		USART6_rxDataHandler(usart6_dma_rxbuf);
		/* 清空接收缓冲区 */
		memset(usart6_dma_rxbuf, 0, USART6_RX_BUF_LEN);

		/* 重新使能DMA接收 */
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
  * @brief  启动DMA双缓冲传输（无中断版本）
  *         自定义的双缓冲DMA启动函数，不使用TC中断（因为使用空闲中断替代）
  * @param  hdma:              DMA句柄指针
  * @param  SrcAddress:        源地址（通常是UART数据寄存器DR的地址）
  * @param  DstAddress:        目标地址0（Memory0缓冲区地址）
  * @param  SecondMemAddress:  目标地址1（Memory1缓冲区地址）
  * @param  DataLength:        传输数据长度（字节数）
  * @retval HAL_StatusTypeDef: HAL_OK成功，HAL_ERROR参数错误，HAL_BUSY状态忙
  * @note   与HAL库DMAEx_MultiBufferStart_IT的区别：
  *         本函数不使能DMA传输完成中断(TC中断)，而是配合空闲中断使用，
  *         减少中断频率，提高系统效率
  */
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* 双缓冲模式不支持内存到内存传输 */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }

	/* 设置DMA传输完成回调函数 */
	/* Memory0传输完成回调 */
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/* Memory1传输完成回调 */
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;

	/* 检查回调函数是否有效 */
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
		hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
		return HAL_ERROR;
	}

	/* 加锁，防止并发操作 */
	__HAL_LOCK(hdma);

	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* 设置DMA状态为忙 */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* 清除错误码 */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* 使能双缓冲模式（设置DMA_SxCR_DBM位） */
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

		/* 配置Memory1的目标地址 */
		hdma->Instance->M1AR = SecondMemAddress;

		/* 配置DMA传输数据长度 */
		hdma->Instance->NDTR = DataLength;

		/* 根据传输方向配置源地址和目标地址 */
		/* 外设到内存（UART接收） */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			hdma->Instance->PAR = DstAddress;       /* 外设地址 = UART数据寄存器 */
			hdma->Instance->M0AR = SrcAddress;      /* 内存0地址 = 接收缓冲区0 */
		}
		/* 内存到外设（UART发送） */
		else
		{
			hdma->Instance->PAR = SrcAddress;        /* 外设地址 = UART数据寄存器 */
			hdma->Instance->M0AR = DstAddress;      /* 内存0地址 = 发送缓冲区 */
		}

		/* 清除传输完成标志 */
		__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* 注意：不使能TC中断，因为使用空闲中断替代 */

		/* 使能DMA */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* DMA不在就绪状态，解锁并返回忙状态 */
		__HAL_UNLOCK(hdma);
		status = HAL_BUSY;
	}

	/* 解锁 */
	__HAL_UNLOCK(hdma);

	return status;
}

/**
  * @brief  启动DMA普通传输（单缓冲区模式）
  * @param  hdma:        DMA句柄指针
  * @param  SrcAddress:  源地址
  * @param  DstAddress:  目标地址
  * @param  DataLength:  传输数据长度
  * @retval HAL_StatusTypeDef: 操作结果
  * @note   用于USART1和USART6的DMA接收启动
  */
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;

	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* 设置DMA状态为忙 */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* 清除错误码 */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* 清除DBM位（单缓冲模式） */
		hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/* 配置传输数据长度 */
		hdma->Instance->NDTR = DataLength;

		/* 根据传输方向配置地址 */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			hdma->Instance->PAR = DstAddress;
			hdma->Instance->M0AR = SrcAddress;
		}
		else
		{
			hdma->Instance->PAR = SrcAddress;
			hdma->Instance->M0AR = DstAddress;
		}

		/* 使能DMA */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		__HAL_UNLOCK(hdma);
		status = HAL_BUSY;
	}
	return status;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  串口中断统一入口函数
  *         在USARTx_IRQHandler()中断服务函数中调用此函数
  * @param  huart: 触发中断的串口句柄
  * @retval 无
  * @note   本函数检测空闲中断(IDLE)标志，检测到后调用空闲中断回调函数处理数据
  *         调用方式：在stm32f4xx_it.c的USARTx_IRQHandler中调用 DRV_UART_IRQHandler(&huartx)
  */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	/* 检查是否为空闲中断（IDLE标志置位且IDLE中断使能） */
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);  /* 调用空闲中断回调函数处理接收数据 */
	}
}

/**
  * @brief  USART1初始化 - 配置空闲中断和DMA接收
  * @param  无
  * @retval 无
  * @note   USART1通常用于：
  *         - 调试串口打印
  *         - IMU传感器数据接收(如BMI088)
  *         使用普通DMA单缓冲模式
  */
void USART1_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);   /* 清除空闲中断标志 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  /* 使能空闲中断 */

	/* 使能UART的DMA接收请求 */
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

	/* 启动DMA接收：外设DR地址 -> usart1_dma_rxbuf缓冲区 */
	DMA_Start(huart1.hdmarx, \
			  (uint32_t)&huart1.Instance->DR, \
			  (uint32_t)usart1_dma_rxbuf, \
			  USART1_RX_BUF_LEN);
}

/**
  * @brief  USART3初始化 - 配置空闲中断和DMA双缓冲接收
  * @param  无
  * @retval 无
  * @note   USART3通常用于接收遥控器(DBUS)数据
  *         使用DMA双缓冲模式，确保在处理数据的同时DMA继续接收
  *         遥控器数据帧固定18字节，使用空闲中断标识一帧结束
  */
void USART3_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);   /* 清除空闲中断标志 */
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  /* 使能空闲中断 */

	/* 使能UART的DMA接收请求 */
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

	/* 启动DMA双缓冲接收：
	 * Memory0: usart3_dma_rxbuf[0]
	 * Memory1: usart3_dma_rxbuf[1]
	 * 传输长度: USART3_RX_DATA_FRAME_LEN(18字节)
	 * 两个缓冲区交替使用，当一个满了自动切换到另一个 */
	DMAEx_MultiBufferStart_NoIT(huart3.hdmarx, \
							    (uint32_t)&huart3.Instance->DR, \
							    (uint32_t)usart3_dma_rxbuf[0], \
							    (uint32_t)usart3_dma_rxbuf[1], \
							    USART3_RX_DATA_FRAME_LEN);
}

/**
  * @brief  USART6初始化 - 配置空闲中断和DMA接收
  * @param  无
  * @retval 无
  * @note   USART6通常用于裁判系统数据接收
  *         使用普通DMA单缓冲模式，缓冲区100字节
  */
void USART6_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);   /* 清除空闲中断标志 */
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  /* 使能空闲中断 */

	/* 使能UART的DMA接收请求 */
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

	/* 启动DMA接收：外设DR地址 -> usart6_dma_rxbuf缓冲区 */
	DMA_Start(huart6.hdmarx, \
			  (uint32_t)&huart6.Instance->DR, \
			  (uint32_t)usart6_dma_rxbuf, \
			  USART6_RX_BUF_LEN);
}

/**
  * @brief  串口格式化打印函数（类似printf）
  * @param  format: 格式化字符串，支持标准printf格式
  * @retval 无
  * @note   通过USART6发送格式化字符串，波特率要求115200
  *         使用可变参数(va_list)实现printf风格的接口
  *         缓冲区固定100字节，注意不要超长
  * @example UART_printf("samples:%f, %f, %f, %f\n", sin(t), sin(2*t), sin(3*t), sin(4*t));
  */
void UART_printf(char *format, ...)
{
    char String[100];  /* 格式化字符串缓冲区（100字节限制） */
    va_list arg;       /* 可变参数列表 */
    va_start(arg, format);           /* 初始化可变参数列表 */
    vsprintf(String, format, arg);   /* 将格式化字符串写入缓冲区 */
    va_end(arg);                     /* 结束可变参数处理 */

    /* 通过USART6发送字符串，超时100ms */
    HAL_UART_Transmit(&huart6, (uint8_t *)String, strlen(String), 100);
}


/* rxData Handler [Weak] functions -------------------------------------------*/

/**
  * @brief  [__WEAK] USART1接收数据处理回调函数 - 弱定义，需要在协议层中重写
  * @param  rxBuf: 指向DMA接收缓冲区的指针，包含接收到的数据
  * @retval 无
  * @note   当USART1接收到一帧完整数据后由空闲中断回调调用
  *         在协议层重写此函数以解析IMU或调试数据
  */
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf)
{
	/* 空实现，等待协议层重写 */
}

/**
  * @brief  [__WEAK] USART3接收数据处理回调函数 - 弱定义，需要在协议层中重写
  * @param  rxBuf: 指向DMA接收缓冲区的指针，包含18字节遥控器数据
  * @retval 无
  * @note   当USART3接收到一帧完整的遥控器(DBUS)数据后调用
  *         在协议层重写此函数以解析遥控器遥控值、按键信息等
  */
__WEAK void USART3_rxDataHandler(uint8_t *rxBuf)
{
	/* 空实现，等待协议层重写 */
}

/**
  * @brief  [__WEAK] USART6接收数据处理回调函数 - 弱定义，需要在协议层中重写
  * @param  rxBuf: 指向DMA接收缓冲区的指针，包含裁判系统数据
  * @retval 无
  * @note   当USART6接收到一帧完整数据后调用
  *         在协议层重写此函数以解析裁判系统反馈(电压、电流、伤害等)
  */
__WEAK void USART6_rxDataHandler(uint8_t *rxBuf)
{
	/* 空实现，等待协议层重写 */
}

