/**
  ******************************************************************************
  * @file    drv_can.c
  * @brief   CAN总线底层驱动 - 提供CAN1/CAN2的收发功能
  *          本文件实现了基于STM32 HAL库的CAN总线底层驱动，包括：
  *          1. CAN滤波器配置与初始化
  *          2. CAN数据接收中断处理（基于FIFO0）
  *          3. 多种CAN数据帧发送函数（标准帧200/1FF/2FF/自定义ID）
  *          4. int16/uint8数据类型转换工具函数
  *          5. 弱定义的接收数据处理回调函数，供协议层重写
  ******************************************************************************
  * @attention
  *
  * Copyright 2024 RobotPilots
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

/* Exported variables --------------------------------------------------------*/
/**
  * CAN总线发送数据缓冲区
  * CAN1_200_DATA: CAN1总线上ID=0x200的8字节发送缓冲区，用于控制电机组1(电机ID 1~4)
  * CAN1_1FF_DATA: CAN1总线上ID=0x1FF的8字节发送缓冲区，用于控制电机组2(电机ID 5~8)
  * CAN1_2FF_DATA: CAN1总线上ID=0x2FF的8字节发送缓冲区，用于扩展电机控制
  * CAN2_xxx_DATA: CAN2总线同理
  */
uint8_t CAN1_200_DATA[8] = {0};
uint8_t CAN1_1FF_DATA[8] = {0};
uint8_t CAN1_2FF_DATA[8] = {0};
uint8_t CAN2_200_DATA[8] = {0};
uint8_t CAN2_1FF_DATA[8] = {0};
uint8_t CAN2_2FF_DATA[8] = {0};

/* Private function prototypes -----------------------------------------------*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

/* Private variables ---------------------------------------------------------*/
/**
  * @brief CAN1/CAN2外设句柄实例(在HAL库中定义)
  */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * CAN接收帧结构体，用于存储接收到的CAN帧数据
  * hcan1RxFrame: CAN1的接收帧
  * hcan2RxFrame: CAN2的接收帧
  */
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;

/**
  * CAN发送头结构体，用于通用的CAN发送配置
  */
CAN_TxHeaderTypeDef CAN_TxHeadeType;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  CAN接收中断回调函数 - 当FIFO0中有新消息挂起时由HAL库自动调用
  *         本函数重写了HAL库中的弱定义回调函数
  * @param  hcan: 触发中断的CAN句柄指针，用于区分是CAN1还是CAN2
  * @retval 无
  * @note   处理流程：
  *         1. 判断是哪个CAN实例触发的中断
  *         2. 从FIFO0读取接收到的CAN帧(包括帧头和数据)
  *         3. 调用对应CAN实例的数据处理函数进行协议解析
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* 判断是CAN1还是CAN2触发的接收中断 */
  if(hcan == &hcan1)
  {
		/* 从CAN1的FIFO0中读取接收到的帧头和数据 */
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
		/* 将接收到的标准ID和数据传递给CAN1的数据处理函数 */
		CAN1_rxDataHandler(hcan1RxFrame.header.StdId, hcan1RxFrame.data);
  }
  else if(hcan == &hcan2)
  {
		/* 从CAN2的FIFO0中读取接收到的帧头和数据 */
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		/* 将接收到的标准ID和数据传递给CAN2的数据处理函数 */
		CAN2_rxDataHandler(hcan2RxFrame.header.StdId, hcan2RxFrame.data);
  }
  else
  {
    return; /* 未知CAN实例，直接返回 */
  }
}

/**
  * @brief  通过CAN1发送标准ID为0x200的数据帧
  *         0x200帧通常用于控制ID为1~4的电机
  * @param  无（数据从全局数组 CAN1_200_DATA 中获取）
  * @retval 无
  */
void CAN1_CMD_200()
{
	CAN_TxHeaderTypeDef tx_message;  /* 发送帧头配置结构体 */

	uint32_t send_mail_box;  /* 发送邮箱号，由HAL库自动分配 */

	/* 配置CAN帧头参数 */
	tx_message.StdId = 0x200;            /* 标准ID: 0x200 */
	tx_message.IDE = CAN_ID_STD;         /* 使用标准帧格式 */
	tx_message.RTR = CAN_RTR_DATA;       /* 数据帧（非远程帧） */
	tx_message.DLC = 0x08;              /* 数据长度: 8字节 */

	/* 将数据添加到发送邮箱，由硬件自动发送 */
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, CAN1_200_DATA, &send_mail_box);
}

/**
  * @brief  通过CAN2发送标准ID为0x200的数据帧
  *         0x200帧通常用于控制ID为1~4的电机
  * @param  无（数据从全局数组 CAN2_200_DATA 中获取）
  * @retval 无
  */
void CAN2_CMD_200()
{
	CAN_TxHeaderTypeDef tx_message;

	uint32_t send_mail_box;
	tx_message.StdId = 0x200;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan2, &tx_message, CAN2_200_DATA, &send_mail_box);
}

/**
  * @brief  通过CAN1发送标准ID为0x1FF的数据帧
  *         0x1FF帧通常用于控制ID为5~8的电机
  * @param  无（数据从全局数组 CAN1_1FF_DATA 中获取）
  * @retval 无
  */
void CAN1_CMD_1FF()
{
	CAN_TxHeaderTypeDef tx_message;

	uint32_t send_mail_box;
	tx_message.StdId = 0x1FF;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan1, &tx_message, CAN1_1FF_DATA, &send_mail_box);
}

/**
  * @brief  通过CAN2发送标准ID为0x1FF的数据帧
  *         0x1FF帧通常用于控制ID为5~8的电机
  * @param  无（数据从全局数组 CAN2_1FF_DATA 中获取）
  * @retval 无
  */
void CAN2_CMD_1FF()
{
	CAN_TxHeaderTypeDef tx_message;

	uint32_t send_mail_box;
	tx_message.StdId = 0x1FF;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan2, &tx_message, CAN2_1FF_DATA, &send_mail_box);
}

/**
  * @brief  将int16类型数组转换为uint8类型数组（大端字节序）
  *         用于将4个int16电机控制值拆分为8个字节，适配CAN总线8字节数据帧格式
  * @param  data: 输出的uint8数组，长度至少为8
  * @param  dat:  输入的int16数组，长度至少为4
  * @retval 无
  * @note   数据排列方式：data[0]=dat[0]高字节, data[1]=dat[0]低字节, ...以此类推
  *         示例: dat[0]=0x1234 -> data[0]=0x12, data[1]=0x34
  */
void int16_to_uint8(uint8_t *data, int16_t *dat)
{
	/* 将每个int16数据拆分为高8位和低8位，按大端序排列 */
	data[0] = (uint8_t)((int16_t)dat[0] >> 8);  /* dat[0] 高字节 */
	data[1] = (uint8_t)((int16_t)dat[0]);         /* dat[0] 低字节 */
	data[2] = (uint8_t)((int16_t)dat[1] >> 8);  /* dat[1] 高字节 */
	data[3] = (uint8_t)((int16_t)dat[1]);         /* dat[1] 低字节 */
	data[4] = (uint8_t)((int16_t)dat[2] >> 8);  /* dat[2] 高字节 */
	data[5] = (uint8_t)((int16_t)dat[2]);         /* dat[2] 低字节 */
	data[6] = (uint8_t)((int16_t)dat[3] >> 8);  /* dat[3] 高字节 */
	data[7] = (uint8_t)((int16_t)dat[3]);         /* dat[3] 低字节 */
}

/**
  * @brief  通过CAN1发送自定义标准ID的数据帧
  * @param  StdId:    CAN标准帧ID（11位）
  * @param  CAN_DATA: 指向8字节发送数据的指针
  * @retval 无
  * @note   适用于发送自定义ID的CAN帧，如电机反馈帧等
  */
void CAN1_SendData(uint32_t StdId, uint8_t *CAN_DATA)
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef tx_message;
	tx_message.IDE = CAN_ID_STD;       /* 标准帧格式 */
	tx_message.RTR = CAN_RTR_DATA;     /* 数据帧 */
	tx_message.DLC = 0x08;            /* 8字节数据 */
	tx_message.StdId = StdId;          /* 自定义标准ID */

	HAL_CAN_AddTxMessage(&hcan1, &tx_message, CAN_DATA, &send_mail_box);
}

/**
  * @brief  通过CAN2发送自定义标准ID的数据帧
  * @param  StdId:    CAN标准帧ID（11位）
  * @param  CAN_DATA: 指向8字节发送数据的指针
  * @retval 无
  */
void CAN2_SendData(uint32_t StdId, uint8_t *CAN_DATA)
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = StdId;

	HAL_CAN_AddTxMessage(&hcan2, &tx_message, CAN_DATA, &send_mail_box);
}

/**
  * @brief  通用CAN发送函数，自动判断使用CAN1还是CAN2
  * @param  hcan:   CAN句柄指针，指向hcan1或hcan2
  * @param  stdId:  CAN标准帧ID（11位）
  * @param  dat:    指向8字节发送数据的指针
  * @retval HAL_StatusTypeDef: HAL_OK表示发送成功，HAL_ERROR表示参数错误或发送失败
  * @note   推荐使用此函数替代CAN1_SendData/CAN2_SendData，可移植性更好
  */
HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)
{
	CAN_TxHeaderTypeDef pHeader;  /* CAN发送帧头 */
	uint32_t txMailBox;            /* 发送邮箱号 */

	/* 检查CAN实例是否合法（只支持CAN1和CAN2） */
	if((hcan->Instance != CAN1)&&(hcan->Instance != CAN2))
	{
		return HAL_ERROR;
	}

	/* 配置CAN帧头参数 */
	pHeader.StdId = stdId;           /* 标准ID */
	pHeader.IDE = CAN_ID_STD;       /* 标准帧格式 */
	pHeader.RTR = CAN_RTR_DATA;     /* 数据帧 */
	pHeader.DLC = 8;                /* 固定8字节数据长度 */

	/* 将数据添加到发送邮箱 */
	if(HAL_CAN_AddTxMessage(hcan, &pHeader, dat, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;  /* 发送失败 */
	}

	return HAL_OK;  /* 发送成功 */
}

/* rxData Handler [Weak] functions -------------------------------------------*/

/**
  * @brief  [__WEAK] CAN1接收数据处理回调函数 - 弱定义，需要在协议层(Protocol Layer)中重写
  * @param  rxId:  接收到的CAN标准帧ID
  * @param  rxBuf: 指向接收到的8字节数据缓冲区
  * @retval 无
  * @note   在drv_can.c中为空实现，实际的数据解析逻辑在协议层实现
  *         根据rxId区分不同电机或传感器的反馈数据
  */
__WEAK void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	/* 空实现，等待协议层重写 */
}

/**
  * @brief  [__WEAK] CAN2接收数据处理回调函数 - 弱定义，需要在协议层(Protocol Layer)中重写
  * @param  rxId:  接收到的CAN标准帧ID
  * @param  rxBuf: 指向接收到的8字节数据缓冲区
  * @retval 无
  */
__WEAK void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	/* 空实现，等待协议层重写 */
}

/**
  * @brief  初始化CAN发送头结构体的配置参数
  * @param  ID: CAN标准帧ID
  * @retval 无
  * @note   将全局CAN_TxHeadeType结构体配置为指定ID的标准数据帧
  */
void HAL_CAN_TxHeadeInit(uint16_t ID)
{
	CAN_TxHeadeType.StdId = ID;            /* 标准ID */
	CAN_TxHeadeType.ExtId = 0x0000;        /* 扩展ID清零（标准帧不使用） */
	CAN_TxHeadeType.DLC = 8;              /* 数据长度8字节 */
	CAN_TxHeadeType.IDE = CAN_ID_STD;      /* 标准帧格式 */
	CAN_TxHeadeType.RTR = CAN_RTR_DATA;    /* 数据帧 */
}


/**
  * @brief  CAN总线滤波器初始化 - 配置CAN1和CAN2的滤波器并启动CAN
  * @param  无
  * @retval 无
  * @note   配置说明：
  *         - 使用掩码模式(ID Mask Mode)，32位滤波器
  *         - 滤波器ID和掩码均设为0x0000，即不做过滤（接收所有ID的帧）
  *         - CAN1使用过滤器组0~13，CAN2使用过滤器组14~27
  *         - 接收到的数据存入FIFO0
  *         - 使能FIFO0消息挂起中断，用于触发接收回调
  */
void CAN_Filter_Init(void)
{
	/* 初始化CAN1和CAN2外设 */
	HAL_CAN_Init(&hcan1);
	HAL_CAN_Init(&hcan2);

	/* 配置CAN滤波器参数 */
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      /* 接收数据存入FIFO0 */
	can_filter_st.FilterActivation = ENABLE;                 /* 使能滤波器 */
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;        /* 掩码过滤模式 */
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;       /* 32位滤波器宽度 */
	can_filter_st.FilterIdHigh = 0x0000;                     /* 滤波器ID高16位 */
	can_filter_st.FilterIdLow = 0x0000;                      /* 滤波器ID低16位 */
	can_filter_st.FilterMaskIdHigh = 0x0000;                 /* 滤波器掩码高16位 */
	can_filter_st.FilterMaskIdLow = 0x0000;                  /* 滤波器掩码低16位 */
	/* 注意: 掩码全为0意味着不做过滤，接收所有ID的CAN帧 */

	/* ---------- 配置CAN1滤波器 ---------- */
	can_filter_st.FilterBank = 0;                            /* CAN1使用过滤器组0开始 */
	can_filter_st.SlaveStartFilterBank = 14;                 /* CAN2从过滤器组14开始（CAN2为CAN1的从机） */
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);           /* 应用滤波器配置到CAN1 */
	HAL_CAN_Start(&hcan1);                                   /* 启动CAN1 */
	/* 使能CAN1的FIFO0消息挂起中断，即接收到新消息时触发中断 */
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* ---------- 配置CAN2滤波器 ---------- */
	can_filter_st.FilterBank = 14;                           /* CAN2使用过滤器组14 */
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);           /* 应用滤波器配置到CAN2 */
	HAL_CAN_Start(&hcan2);                                   /* 启动CAN2 */
	/* 使能CAN2的FIFO0消息挂起中断 */
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
