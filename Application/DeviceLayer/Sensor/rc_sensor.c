/**
  * @file        rc_sensor.c
  * @author      RobotPilots@2020
  * @Version     V1.0
  * @date        9-September-2020
  * @brief       遥控器传感器驱动模块
  * @details     处理遥控器数据的解析、校验和状态管理。
  *              支持SBUS协议遥控器（通道数据、拨轮、开关等）以及键鼠数据。
  *              包含数据越界检查、拨轮阶跃检测和心跳包检测功能。
  */

/* Includes ------------------------------------------------------------------*/
#include "rc_sensor.h"
#include "rp_math.h"

/* 外部声明遥控器初始化和更新函数（在其他文件中实现） */
extern void rc_sensor_init(rc_sensor_t *rc_sen);
extern void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf);

/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_check(rc_sensor_t *rc_sen);       /* 遥控器数据校验 */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen);   /* 遥控器心跳检测 */

/* Exported variables --------------------------------------------------------*/

/**
  * @brief  遥控器UART驱动对象
  * @details 指定使用UART2作为遥控器通信接口，无发送功能（只接收）
  */
drv_uart_t	rc_sensor_driver = {
	.id = DRV_UART2,    /* 使用UART2 */
	.tx_byte = NULL,    /* 遥控器只接收，不发送 */
};

/**
  * @brief  遥控器信息全局变量
  * @details 存储拨轮阶跃检测的阈值和离线检测参数
  *          拨轮阶跃值定义了四个档位的触发阈值（按绝对值从大到小排列）：
  *          - RC_TB_UP: 拨轮推到顶部（负方向最大值-650）
  *          - RC_TB_MU: 拨轮往上推一点（负方向小值-200）
  *          - RC_TB_DN: 拨轮推到底部（正方向最大值+650）
  *          - RC_TB_MD: 拨轮往下推一点（正方向小值+200）
  */
rc_sensor_info_t 	rc_sensor_info = {
	/* 拨轮阶跃检测阈值 */
	.tw_step_value[RC_TB_UP] = -650,  /* 上拨（顶） */
	.tw_step_value[RC_TB_MU] = -200,  /* 上拨（微） */
	.tw_step_value[RC_TB_DN] = +650,  /* 下拨（底） */
	.tw_step_value[RC_TB_MD] = +200,  /* 下拨（微） */
	/* 离线检测：超过60个周期未收到数据则判定离线 */
	.offline_max_cnt = 60,
};

/**
  * @brief  遥控器传感器对象
  * @details 绑定所有操作函数和状态信息
  */
rc_sensor_t	rc_sensor = {
	.info = &rc_sensor_info,           /* 指向遥控器信息 */
	.init = rc_sensor_init,            /* 初始化函数 */
	.update = rc_sensor_update,        /* 数据更新函数 */
	.check = rc_sensor_check,          /* 数据校验函数 */
	.heart_beat = rc_sensor_heart_beat, /* 心跳检测函数 */
	.work_state = DEV_OFFLINE,         /* 初始状态为离线 */
	.id = DEV_ID_RC,                   /* 设备ID */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  遥控器数据校验与拨轮阶跃检测
  * @param  rc_sen: 遥控器传感器对象指针
  * @retval None
  * @details 功能分为两部分：
  *          1. 拨轮阶跃检测：检测拨轮从中间位置推到各档位并回中的过程，
  *             在回中时根据推过的最大幅度判断触发了哪个档位，输出阶跃信号
  *          2. 通道数据校验：检查四个摇杆通道数据是否越界（有效范围±660），
  *             越界时清零所有数据并报错
  *
  * 拨轮档位说明：
  *   step[0](RC_TB_UP): 拨轮推到顶部（-650以上）
  *   step[1](RC_TB_MU): 拨轮微微上推（-200以上）
  *   step[2](RC_TB_DN): 拨轮推到底部（+650以上）
  *   step[3](RC_TB_MD): 拨轮微微下推（+200以上）
  */
static void rc_sensor_check(rc_sensor_t *rc_sen)
{
	/* 记录拨轮推过的最大绝对值，用于回中时判断档位 */
	static int16_t thumbwheel_record = 0;

	/* 记录上次拨轮顶/底档位状态，用于检测状态变化 */
	static uint8_t step_record[2] = {0, 0};

	rc_sensor_info_t *rc_info = rc_sen->info;

	/* ==================== 拨轮阶跃检测 ==================== */
	/* 检测拨轮是否正在被推向更远处（绝对值增大），
	   如果是则更新记录的最大值 */
	if((abs(rc_info->thumbwheel.value_last) < abs(rc_info->thumbwheel.value)) && \
		 (abs(thumbwheel_record) < abs(rc_info->thumbwheel.value)))
	{
		thumbwheel_record = rc_info->thumbwheel.value;  /* 记录当前最大幅度 */
	}

	/* 检测拨轮是否回到中间位置（绝对值≤10），且之前有推过记录 */
	if((abs(rc_info->thumbwheel.value) <=10) && (thumbwheel_record != 0))
	{
		/* 遍历4个档位阈值，判断推过的幅度是否达到对应档位 */
		for(char i = 0; i < 4; i++)
		{
			/* 正方向档位检测（下推） */
			if(rc_info->tw_step_value[i] > 0 && thumbwheel_record > 0)
			{
				if(thumbwheel_record >= rc_info->tw_step_value[i])
				{
					rc_info->thumbwheel.step[i] = !rc_info->thumbwheel.step[i];  /* 翻转阶跃状态 */
					thumbwheel_record = 0; //清零操作！！
				}
			}
			/* 负方向档位检测（上推） */
			if(rc_info->tw_step_value[i] < 0 && thumbwheel_record < 0)
			{
				if(thumbwheel_record <= rc_info->tw_step_value[i])
				{
					rc_info->thumbwheel.step[i] = !rc_info->thumbwheel.step[i];  /* 翻转阶跃状态 */
					thumbwheel_record = 0;//清零操作！！
				}
			}
		}
		/* 清零记录值 */
		thumbwheel_record = 0;
	}

	/* 保存当前拨轮值作为下次比较的历史值 */
	rc_info->thumbwheel.value_last = rc_info->thumbwheel.value;

	/* 检测顶/底档位的状态变化（上升沿/下降沿检测） */
	if(rc_info->thumbwheel.step[RC_TB_UP] != step_record[0])
	{
		rc_info->thumbwheel.step_change[RC_MD_TO_UP] = 1;  /* 标记状态变化 */
	}
	else
	{
		rc_info->thumbwheel.step_change[RC_MD_TO_UP] = 0;  /* 无变化 */
	}
	step_record[0] = rc_info->thumbwheel.step[RC_TB_UP];

  	/* 检测底档位的状态变化（向下拨） */
  if(rc_info->thumbwheel.step[RC_TB_DN] != step_record[1])
    {
      rc_info->thumbwheel.step_change[RC_MD_TO_DO] = 1;  /* 标记向下状态变化 */
    }
    else
    {
      rc_info->thumbwheel.step_change[RC_MD_TO_DO] = 0;  /* 无变化 */
    }
    step_record[1] = rc_info->thumbwheel.step[RC_TB_DN];
  
    
	/* ==================== 通道数据越界检查 ==================== */
	/* SBUS协议有效通道范围为±660，超出此范围说明数据异常 */
	if(abs(rc_info->ch0) > 660 ||
	   abs(rc_info->ch1) > 660 ||
	   abs(rc_info->ch2) > 660 ||
	   abs(rc_info->ch3) > 660)
	{
		/* 数据异常：设置错误码，清零所有通道数据 */
		rc_sen->errno = DEV_DATA_ERR;
		rc_info->ch0 = 0;
		rc_info->ch1 = 0;
		rc_info->ch2 = 0;
		rc_info->ch3 = 0;
		rc_info->s1 = RC_SW_MID;    /* 开关恢复中位 */
		rc_info->s2 = RC_SW_MID;
		rc_info->thumbwheel.value = 0;         /* 拨轮清零 */
		rc_info->thumbwheel.value_last = 0;
		rc_info->thumbwheel.step[RC_TB_UP] = 0;
		rc_info->thumbwheel.step[RC_TB_MU] = 0;
		rc_info->thumbwheel.step[RC_TB_MD] = 0;
		rc_info->thumbwheel.step[RC_TB_DN] = 0;
	}
	else
	{
		/* 数据正常 */
		rc_sen->errno = NONE_ERR;
	}
}

/**
  * @brief  遥控器心跳检测函数
  * @param  rc_sen: 遥控器传感器对象指针
  * @retval None
  * @details 每次调用离线计数器加1，超过最大阈值则判定离线。
  *          当收到有效数据时（在update中），offline_cnt会被清零。
  */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen)
{
	rc_sensor_info_t *rc_info = rc_sen->info;

	/* 离线计数器递增 */
	rc_info->offline_cnt++;

	/* 超过最大离线计数阈值，判定设备离线 */
	if(rc_info->offline_cnt > rc_info->offline_max_cnt) {
		rc_info->offline_cnt = rc_info->offline_max_cnt;  /* 钳位防止溢出 */
		rc_sen->work_state = DEV_OFFLINE;
	}
	else {
		/* 未超阈值且当前状态为离线，则恢复在线（说明收到了数据） */
		if(rc_sen->work_state == DEV_OFFLINE)
		{
			rc_sen->work_state = DEV_ONLINE;
		}
	}
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  检查遥控器所有通道是否处于中位（死区范围内）
  * @retval true: 所有通道都在中位死区内
  *         false: 至少有一个通道不在中位
  * @details 使用DeathZoom函数判断通道值是否在0±50的死区范围内
  */
bool RC_IsChannelReset(void)
{
	if(  (DeathZoom(rc_sensor_info.ch0, 0, 50) == 0) &&
		 (DeathZoom(rc_sensor_info.ch1, 0, 50) == 0) &&
		 (DeathZoom(rc_sensor_info.ch2, 0, 50) == 0) &&
		 (DeathZoom(rc_sensor_info.ch3, 0, 50) == 0))
	{
		return true;
	}
	return false;
}

/**
  * @brief  重置遥控器所有数据到默认值
  * @param  rc: 遥控器传感器对象指针
  * @retval None
  * @details 将所有通道、开关、鼠标、键盘和拨轮数据清零/恢复默认值，
  *          用于系统初始化或紧急情况下的数据复位
  */
void RC_ResetData(rc_sensor_t *rc)
{
	/* 摇杆通道清零（中位状态） */
	rc->info->ch0 = 0;
	rc->info->ch1 = 0;
	rc->info->ch2 = 0;
	rc->info->ch3 = 0;

	/* 左右拨杆开关恢复中间位置 */
	rc->info->s1 = RC_SW_MID;
	rc->info->s2 = RC_SW_MID;

	/* 鼠标数据清零 */
	rc->info->mouse_vx = 0;     /* 鼠标X轴速度 */
	rc->info->mouse_vy = 0;     /* 鼠标Y轴速度 */
	rc->info->mouse_vz = 0;     /* 鼠标Z轴速度 */
	rc->info->mouse_x = 0.f;    /* 鼠标X轴位移 */
	rc->info->mouse_y = 0.f;    /* 鼠标Y轴位移 */
	rc->info->mouse_z = 0.f;    /* 鼠标Z轴位移 */
	rc->info->mouse_btn_l.value = 0;  /* 左键 */
	rc->info->mouse_btn_r.value = 0;  /* 右键 */

	/* 键盘按键清零 */
	rc->info->key_v = 0;        /* 键盘值 */
	rc->info->W.value = 0;
	rc->info->S.value = 0;
	rc->info->A.value = 0;
	rc->info->D.value = 0;
	rc->info->Shift.value = 0;
	rc->info->Ctrl.value = 0;
	rc->info->Q.value = 0;
	rc->info->E.value = 0;
	rc->info->R.value = 0;
	rc->info->F.value = 0;
	rc->info->G.value = 0;
	rc->info->Z.value = 0;
	rc->info->X.value = 0;
	rc->info->C.value = 0;
	rc->info->V.value = 0;
	rc->info->B.value = 0;

	/* 左拨轮清零 */
	rc->info->thumbwheel.value = 0;
	rc->info->thumbwheel.value_last = 0;
	rc->info->thumbwheel.step[RC_TB_UP] = 0;
	rc->info->thumbwheel.step[RC_TB_MU] = 0;
	rc->info->thumbwheel.step[RC_TB_MD] = 0;
	rc->info->thumbwheel.step[RC_TB_DN] = 0;
}
