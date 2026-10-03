/**
 * @file    motor.c
 * @brief   电机驱动层 - 多种电机类型的初始化和管理
 *
 * 本文件实现了多种电机的配置、初始化和列表管理功能，支持以下电机类型：
 *   1. RM（RoboMaster）电机 - 如 6020 云台电机，使用 CAN 总线通信
 *   2. KT 系列电机 - 如 KT9015 驱动电机，使用 CAN 总线通信
 *   3. HT 系列电机 - 轮毂电机，使用 CAN 总线通信
 *   4. DM 系列电机 - 达妙伺服电机，使用 CAN 总线通信
 *
 * 每种电机类型均采用面向对象风格封装（结构体 + 函数指针），
 * 包含 born_info（出厂信息）、rx_info（接收信息）、tx_info（发送信息）、
 * state（状态信息）等子结构体。
 *
 * @note   添加新电机时需要配置：电机类型（3508/6020）、CAN 总线（CAN1/CAN2）、接收 ID
 * @version V1.0
 * @date    2026-06-11
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"  /* 电机驱动头文件，包含所有电机类型定义和函数声明 */
#include "pid_conf.h" /* PID 参数宏，摩擦轮六轮共用 SHOOT_SP_* 一套标定值 */


/* Private variables ---------------------------------------------------------*/
/* ==================== HT 电机定义 ==================== */
/* HT 电机外部 CAN 句柄声明 */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 左轮 HT 电机出厂信息配置
 * stdId: 0x009 - 电机控制报文 ID
 * hcan:  使用 CAN1 总线
 * order_correction: 电机总角度的正方向为顺时针（0表示不反转）
 */
Motor_HT_Born_Info_t L_Wheel_Born_Info =
{
	.stdId = 0x009,             /* 电机控制报文 ID */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.order_correction = 0,      /* 电机正方向：顺时针（0=不反转，1=反转） */
};

/* 左轮 HT 电机接收信息、发送信息、状态信息结构体（初始化时由库自动填充） */
Motor_HT_Rx_Info_t L_Wheel_Rx_Info_t;   /* 接收数据结构体 */
Motor_HT_Tx_Info_t L_Wheel_Tx_Info_t;   /* 发送数据结构体 */
Motor_HT_State_t L_Wheel_State_t;       /* 电机状态结构体 */

/**
 * @brief 左轮 HT 电机完整实例
 * 通过结构体嵌套方式组织所有子模块信息
 * single_init 函数指针指向 HT 单电机初始化函数
 */
Motor_HT_t L_Wheel =
{
	.born_info = &L_Wheel_Born_Info,     /* 出厂信息 */
	.rx_info = &L_Wheel_Rx_Info_t,       /* 接收信息 */
	.tx_info = &L_Wheel_Tx_Info_t,       /* 发送信息 */
	.state = &L_Wheel_State_t,           /* 状态信息 */
	.single_init = &HT_Single_Motor_Init, /* 单电机初始化函数指针 */
};

/* ==================== DM（达妙）电机定义 ==================== */

/**
 * @brief Yaw 轴 DM 电机出厂信息配置
 * DM 电机是达妙伺服电机，支持位置/速度/力矩控制模式
 * stdId: 0x001 - 电机控制报文 ID
 */
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x001,             /* 电机控制报文 ID */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
};

/* Yaw 轴 DM 电机各子模块结构体 */
Motor_DM_Rx_Info_t Yaw_Rx_Info_t;       /* 接收数据结构体 */
Motor_DM_Tx_Info_t Yaw_Tx_Info_t;       /* 发送数据结构体 */
Motor_DM_State_t Yaw_State_t;           /* 电机状态结构体 */

/**
 * @brief Yaw 轴 DM 电机完整实例
 * 用于云台 Yaw 轴的精确角度控制
 */
Motor_DM_t Yaw_Motor =
{
	.born_info = &Yaw_Born_Info,         /* 出厂信息 */
	.rx_info = &Yaw_Rx_Info_t,           /* 接收信息 */
	.tx_info = &Yaw_Tx_Info_t,           /* 发送信息 */
	.state = &Yaw_State_t,               /* 状态信息 */
	.single_init = &DM_Single_Motor_Init, /* 单电机初始化函数指针 */
};

/* ==================== RM（RoboMaster）电机定义 ==================== */
/*
 * 六摩擦轮均为 RM3508，挂在 CAN1，分两组发送：
 *   前层 F_U / F_R / F_L  —— 0x200 组（电机 ID 1~3），反馈 0x201~0x203
 *   后层 B_U / B_R / B_L  —— 0x1FF 组（电机 ID 5~7），反馈 0x205~0x207
 * rxId 为发送帧内槽位号（0~3 对应字节对 0-1 / 2-3 / 4-5 / 6-7），不是电机 ID。
 */

/**
 * @brief 前层上摩擦轮 RM 电机出厂信息配置
 * rxId: 0 - 发送帧内槽位
 * hcan: 使用 CAN1 总线
 * type: _3508_Single - 3508 电机单电机控制模式
 * stdId: 0x200 - 发送报文基地址（对应 ID=1~4 的 3508 电机）
 */
Motor_RM_Born_Info_t F_U_Fric_Born =
{
	.rxId = 0,                  /* 发送帧内槽位 0 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.type = _3508_Single,       /* 电机类型：3508 单电机模式 */
	.stdId = 0x200,             /* 发送报文基地址 */
};

/* 前层上摩擦轮 RM 电机各子模块结构体 */
Motor_RM_Tx_Info_t F_U_Fric_Tx;              /* 发送数据结构体 */
Motor_RM_State_t F_U_Fric_State;             /* 电机状态结构体 */
Motor_RM_Rx_Info_t F_U_Fric_Rx;              /* 接收数据结构体 */

/**
 * @brief 前层上摩擦轮速度环 PID 控制参数
 * 六轮共用 pid_conf.h 的 SHOOT_SP_* 一套标定值
 */
pid_ctrl_t F_U_Fric_Speed_Ctrl =
{
	.kp = SHOOT_SP_KP,          /* 比例系数 */
	.ki = SHOOT_SP_KI,          /* 积分系数 */
	.kd = SHOOT_SP_KD,          /* 微分系数 */
	.integral_max = SHOOT_SP_MAX_INTEGRAL, /* 积分项限幅 */
	.out_max = SHOOT_SP_MAX_OUT,           /* 输出限幅 */
};

/* 前层上摩擦轮控制信息 */
Motor_RM_Ctrl_Info_t F_U_Fric_Ctrl =
{
	.speed_ctrl = &F_U_Fric_Speed_Ctrl,  /* 速度环 PID 控制器 */
};

/**
 * @brief 前层上摩擦轮 RM 电机完整实例
 */
Motor_RM_t F_U_Fric =
{
	.born_info = &F_U_Fric_Born,         /* 出厂信息 */
	.rx_info = &F_U_Fric_Rx,             /* 接收信息 */
	.tx_info = &F_U_Fric_Tx,             /* 发送信息 */
	.state = &F_U_Fric_State,            /* 状态信息 */
	.single_init = RM_Motor_Init,        /* 单电机初始化函数指针 */
	.ctrl = &F_U_Fric_Ctrl,              /* 控制信息（含 PID） */
};

/**
 * @brief 前层右摩擦轮 RM 电机出厂信息配置
 * stdId: 0x200 - 与前层上/左同组发送
 */
Motor_RM_Born_Info_t F_R_Fric_Born =
{
	.rxId = 1,                  /* 发送帧内槽位 1 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.type = _3508_Single,       /* 电机类型：3508 单电机模式 */
	.stdId = 0x200,             /* 发送报文基地址 */
};

/* 前层右摩擦轮 RM 电机各子模块结构体 */
Motor_RM_Tx_Info_t F_R_Fric_Tx;              /* 发送数据结构体 */
Motor_RM_State_t F_R_Fric_State;             /* 电机状态结构体 */
Motor_RM_Rx_Info_t F_R_Fric_Rx;              /* 接收数据结构体 */

/* 前层右摩擦轮速度环 PID 控制参数 */
pid_ctrl_t F_R_Fric_Speed_Ctrl =
{
	.kp = SHOOT_SP_KP,          /* 比例系数 */
	.ki = SHOOT_SP_KI,          /* 积分系数 */
	.kd = SHOOT_SP_KD,          /* 微分系数 */
	.integral_max = SHOOT_SP_MAX_INTEGRAL, /* 积分项限幅 */
	.out_max = SHOOT_SP_MAX_OUT,           /* 输出限幅 */
};

/* 前层右摩擦轮控制信息 */
Motor_RM_Ctrl_Info_t F_R_Fric_Ctrl =
{
	.speed_ctrl = &F_R_Fric_Speed_Ctrl,  /* 速度环 PID 控制器 */
};

/**
 * @brief 前层右摩擦轮 RM 电机完整实例
 */
Motor_RM_t F_R_Fric =
{
	.born_info = &F_R_Fric_Born,         /* 出厂信息 */
	.rx_info = &F_R_Fric_Rx,             /* 接收信息 */
	.tx_info = &F_R_Fric_Tx,             /* 发送信息 */
	.state = &F_R_Fric_State,            /* 状态信息 */
	.single_init = RM_Motor_Init,        /* 单电机初始化函数指针 */
	.ctrl = &F_R_Fric_Ctrl,              /* 控制信息（含 PID） */
};

/**
 * @brief 前层左摩擦轮 RM 电机出厂信息配置
 * stdId: 0x200 - 与前层上/右同组发送
 */
Motor_RM_Born_Info_t F_L_Fric_Born =
{
	.rxId = 2,                  /* 发送帧内槽位 2 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.type = _3508_Single,       /* 电机类型：3508 单电机模式 */
	.stdId = 0x200,             /* 发送报文基地址 */
};

/* 前层左摩擦轮 RM 电机各子模块结构体 */
Motor_RM_Tx_Info_t F_L_Fric_Tx;              /* 发送数据结构体 */
Motor_RM_State_t F_L_Fric_State;             /* 电机状态结构体 */
Motor_RM_Rx_Info_t F_L_Fric_Rx;              /* 接收数据结构体 */

/* 前层左摩擦轮速度环 PID 控制参数 */
pid_ctrl_t F_L_Fric_Speed_Ctrl =
{
	.kp = SHOOT_SP_KP,          /* 比例系数 */
	.ki = SHOOT_SP_KI,          /* 积分系数 */
	.kd = SHOOT_SP_KD,          /* 微分系数 */
	.integral_max = SHOOT_SP_MAX_INTEGRAL, /* 积分项限幅 */
	.out_max = SHOOT_SP_MAX_OUT,           /* 输出限幅 */
};

/* 前层左摩擦轮控制信息 */
Motor_RM_Ctrl_Info_t F_L_Fric_Ctrl =
{
	.speed_ctrl = &F_L_Fric_Speed_Ctrl,  /* 速度环 PID 控制器 */
};

/**
 * @brief 前层左摩擦轮 RM 电机完整实例
 */
Motor_RM_t F_L_Fric =
{
	.born_info = &F_L_Fric_Born,         /* 出厂信息 */
	.rx_info = &F_L_Fric_Rx,             /* 接收信息 */
	.tx_info = &F_L_Fric_Tx,             /* 发送信息 */
	.state = &F_L_Fric_State,            /* 状态信息 */
	.single_init = RM_Motor_Init,        /* 单电机初始化函数指针 */
	.ctrl = &F_L_Fric_Ctrl,              /* 控制信息（含 PID） */
};

/**
 * @brief 后层上摩擦轮 RM 电机出厂信息配置
 * stdId: 0x1FF - 对应 ID=5~8 的 3508 电机
 */
Motor_RM_Born_Info_t B_U_Fric_Born =
{
	.rxId = 0,                  /* 发送帧内槽位 0 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.type = _3508_Single,       /* 电机类型：3508 单电机模式 */
	.stdId = 0x1FF,             /* 发送报文基地址 */
};

/* 后层上摩擦轮 RM 电机各子模块结构体 */
Motor_RM_Tx_Info_t B_U_Fric_Tx;              /* 发送数据结构体 */
Motor_RM_State_t B_U_Fric_State;             /* 电机状态结构体 */
Motor_RM_Rx_Info_t B_U_Fric_Rx;              /* 接收数据结构体 */

/* 后层上摩擦轮速度环 PID 控制参数 */
pid_ctrl_t B_U_Fric_Speed_Ctrl =
{
	.kp = SHOOT_SP_KP,          /* 比例系数 */
	.ki = SHOOT_SP_KI,          /* 积分系数 */
	.kd = SHOOT_SP_KD,          /* 微分系数 */
	.integral_max = SHOOT_SP_MAX_INTEGRAL, /* 积分项限幅 */
	.out_max = SHOOT_SP_MAX_OUT,           /* 输出限幅 */
};

/* 后层上摩擦轮控制信息 */
Motor_RM_Ctrl_Info_t B_U_Fric_Ctrl =
{
	.speed_ctrl = &B_U_Fric_Speed_Ctrl,  /* 速度环 PID 控制器 */
};

/**
 * @brief 后层上摩擦轮 RM 电机完整实例
 */
Motor_RM_t B_U_Fric =
{
	.born_info = &B_U_Fric_Born,         /* 出厂信息 */
	.rx_info = &B_U_Fric_Rx,             /* 接收信息 */
	.tx_info = &B_U_Fric_Tx,             /* 发送信息 */
	.state = &B_U_Fric_State,            /* 状态信息 */
	.single_init = RM_Motor_Init,        /* 单电机初始化函数指针 */
	.ctrl = &B_U_Fric_Ctrl,              /* 控制信息（含 PID） */
};

/**
 * @brief 后层右摩擦轮 RM 电机出厂信息配置
 * stdId: 0x1FF - 与后层上/左同组发送
 */
Motor_RM_Born_Info_t B_R_Fric_Born =
{
	.rxId = 1,                  /* 发送帧内槽位 1 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.type = _3508_Single,       /* 电机类型：3508 单电机模式 */
	.stdId = 0x1FF,             /* 发送报文基地址 */
};

/* 后层右摩擦轮 RM 电机各子模块结构体 */
Motor_RM_Tx_Info_t B_R_Fric_Tx;              /* 发送数据结构体 */
Motor_RM_State_t B_R_Fric_State;             /* 电机状态结构体 */
Motor_RM_Rx_Info_t B_R_Fric_Rx;              /* 接收数据结构体 */

/* 后层右摩擦轮速度环 PID 控制参数 */
pid_ctrl_t B_R_Fric_Speed_Ctrl =
{
	.kp = SHOOT_SP_KP,          /* 比例系数 */
	.ki = SHOOT_SP_KI,          /* 积分系数 */
	.kd = SHOOT_SP_KD,          /* 微分系数 */
	.integral_max = SHOOT_SP_MAX_INTEGRAL, /* 积分项限幅 */
	.out_max = SHOOT_SP_MAX_OUT,           /* 输出限幅 */
};

/* 后层右摩擦轮控制信息 */
Motor_RM_Ctrl_Info_t B_R_Fric_Ctrl =
{
	.speed_ctrl = &B_R_Fric_Speed_Ctrl,  /* 速度环 PID 控制器 */
};

/**
 * @brief 后层右摩擦轮 RM 电机完整实例
 */
Motor_RM_t B_R_Fric =
{
	.born_info = &B_R_Fric_Born,         /* 出厂信息 */
	.rx_info = &B_R_Fric_Rx,             /* 接收信息 */
	.tx_info = &B_R_Fric_Tx,             /* 发送信息 */
	.state = &B_R_Fric_State,            /* 状态信息 */
	.single_init = RM_Motor_Init,        /* 单电机初始化函数指针 */
	.ctrl = &B_R_Fric_Ctrl,              /* 控制信息（含 PID） */
};

/**
 * @brief 后层左摩擦轮 RM 电机出厂信息配置
 * stdId: 0x1FF - 与后层上/右同组发送
 */
Motor_RM_Born_Info_t B_L_Fric_Born =
{
	.rxId = 2,                  /* 发送帧内槽位 2 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.type = _3508_Single,       /* 电机类型：3508 单电机模式 */
	.stdId = 0x1FF,             /* 发送报文基地址 */
};

/* 后层左摩擦轮 RM 电机各子模块结构体 */
Motor_RM_Tx_Info_t B_L_Fric_Tx;              /* 发送数据结构体 */
Motor_RM_State_t B_L_Fric_State;             /* 电机状态结构体 */
Motor_RM_Rx_Info_t B_L_Fric_Rx;              /* 接收数据结构体 */

/* 后层左摩擦轮速度环 PID 控制参数 */
pid_ctrl_t B_L_Fric_Speed_Ctrl =
{
	.kp = SHOOT_SP_KP,          /* 比例系数 */
	.ki = SHOOT_SP_KI,          /* 积分系数 */
	.kd = SHOOT_SP_KD,          /* 微分系数 */
	.integral_max = SHOOT_SP_MAX_INTEGRAL, /* 积分项限幅 */
	.out_max = SHOOT_SP_MAX_OUT,           /* 输出限幅 */
};

/* 后层左摩擦轮控制信息 */
Motor_RM_Ctrl_Info_t B_L_Fric_Ctrl =
{
	.speed_ctrl = &B_L_Fric_Speed_Ctrl,  /* 速度环 PID 控制器 */
};

/**
 * @brief 后层左摩擦轮 RM 电机完整实例
 */
Motor_RM_t B_L_Fric =
{
	.born_info = &B_L_Fric_Born,         /* 出厂信息 */
	.rx_info = &B_L_Fric_Rx,             /* 接收信息 */
	.tx_info = &B_L_Fric_Tx,             /* 发送信息 */
	.state = &B_L_Fric_State,            /* 状态信息 */
	.single_init = RM_Motor_Init,        /* 单电机初始化函数指针 */
	.ctrl = &B_L_Fric_Ctrl,              /* 控制信息（含 PID） */
};

/**
 * @brief 前层摩擦轮电机组
 * 三颗 3508 打包进 0x200 一帧发送，组内槽位与 rxId 一一对应
 */
Motor_RM_Group_t Fric_F_Group =
{
	.motor[0] = &F_U_Fric,      /* 组内第 1 个电机：前层上摩擦轮 */
	.motor[1] = &F_R_Fric,      /* 组内第 2 个电机：前层右摩擦轮 */
	.motor[2] = &F_L_Fric,      /* 组内第 3 个电机：前层左摩擦轮 */
	.motor[3] = NULL,           /* 组内第 4 个电机：未使用 */
	.stdId = 0x200,             /* 组发送报文基地址 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.group_init = RM_Group_Motor_Init,  /* 组初始化函数指针 */
};

/**
 * @brief 后层摩擦轮电机组
 * 三颗 3508 打包进 0x1FF 一帧发送，组内槽位与 rxId 一一对应
 */
Motor_RM_Group_t Fric_B_Group =
{
	.motor[0] = &B_U_Fric,      /* 组内第 1 个电机：后层上摩擦轮 */
	.motor[1] = &B_R_Fric,      /* 组内第 2 个电机：后层右摩擦轮 */
	.motor[2] = &B_L_Fric,      /* 组内第 3 个电机：后层左摩擦轮 */
	.motor[3] = NULL,           /* 组内第 4 个电机：未使用 */
	.stdId = 0x1FF,             /* 组发送报文基地址 */
	.hcan = &hcan1,             /* 使用 CAN1 总线 */
	.group_init = RM_Group_Motor_Init,  /* 组初始化函数指针 */
};

/* ==================== KT 系列电机定义 ==================== */

/**
 * @brief KT 电机数组
 * KT9015 电机支持多种控制模式：
 *   - angle_single_Control: 单圈角度控制
 *   - angle_add_Control: 角度累加控制
 *   - angle_sum_Control: 总角度控制
 *   - iqControl: 电流（力矩）控制
 *   - speedControl: 速度控制
 */
KT_motor_t kt_motor[] = {
	[0] = {
		.KT_motor_info = {
			.tx_info = {
				.angle_single_Control = 0,            /* 单圈角度控制值 */
				.angle_single_Control_maxSpeed = 0,    /* 单圈角度控制最大速度 */
				.angle_single_Control_spinDirection = 0, /* 旋转方向 */
				.angle_add_Control = 0,               /* 角度累加控制值 */
				.angle_add_Control_maxSpeed = 0,       /* 角度累加控制最大速度 */
				.angle_sum_Control = 0,               /* 总角度控制值 */
				.angle_sum_Control_maxSpeed = 0,       /* 总角度控制最大速度 */
				.iqControl = 0,                       /* 电流（力矩）控制值 */
				.speedControl = 0,                    /* 速度控制值 */
			},
			.id = {
				.tx_id = ID_GIMB_YAW,         /* 发送 ID：云台 Yaw 轴 */
				.rx_id = 0x88,                 /* 接收报文 ID */
				.drive_type = M_CAN1,          /* 驱动类型：使用 CAN1 */
				.motor_type = KT9015,          /* 电机型号：KT9015 */
			},
		},
		.init = KT_motor_class_init,  /* KT 电机类初始化函数指针 */
	},
};

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  RM 电机列表初始化
 * 初始化所有已配置的 RoboMaster 电机实例
 * 包括单电机初始化和组初始化
 */
void rm_motor_list_init()
{
	/* 组初始化内部会逐个调用成员的 single_init，无需再单独初始化 */
	Fric_F_Group.group_init(&Fric_F_Group); /* 初始化前层摩擦轮电机组 */
	Fric_B_Group.group_init(&Fric_B_Group); /* 初始化后层摩擦轮电机组 */
}

/**
 * @brief  KT 电机列表初始化
 * 初始化所有已配置的 KT 系列电机实例
 */
void kt_motor_list_init()
{
	kt_motor[0].init(&kt_motor[0]);    /* 初始化第 0 号 KT 电机 */
}

/**
 * @brief  DM 电机列表初始化
 * 初始化所有已配置的达妙电机实例
 */
void dm_motor_list_init()
{
	Yaw_Motor.single_init(&Yaw_Motor); /* 初始化 Yaw 轴 DM 电机 */
}

/**
 * @brief  HT 电机列表初始化
 * 初始化所有已配置的 HT 系列电机实例
 */
void ht_motor_list_init()
{
	L_Wheel.single_init(&L_Wheel);     /* 初始化左轮 HT 电机 */
}

/**
 * @brief  RM 电机组心跳包
 * 定期发送心跳包以维持 RM 电机组的通信连接
 * 在系统主循环或定时任务中调用
 */
void rm_motor_list_heart_beat()
{
	Fric_F_Group.group_heartbeat(&Fric_F_Group); /* 前层摩擦轮心跳失联检测 */
	Fric_B_Group.group_heartbeat(&Fric_B_Group); /* 后层摩擦轮心跳失联检测 */
}
