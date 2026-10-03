/**
 * @file    rc_sensor.c
 * @brief   遥控器传感器管理模块
 *
 * 本文件实现了遥控器（RC）传感器的完整功能，包括：
 *   1. 遥控器数据接收和解析（DBUS 协议）
 *   2. 拨轮跳变检测（支持上下两档跳变）
 *   3. 拨杆状态检测（上/中/下三档）
 *   4. 数据有效性检查（范围校验）
 *   5. 心跳包（离线检测）
 *   6. 数据复位（安全保护）
 *
 * 遥控器输入包括：
 *   - 4 个摇杆通道（ch0-ch3）
 *   - 2 个拨杆开关（s1, s2，各三档）
 *   - 1 个拨轮（支持跳变检测）
 *   - 鼠标（移动速度、按键）
 *   - 键盘（WASD 等按键）
 *
 * @note   遥控器通过 UART3 以 DBUS 协议接收数据，波特率 100000
 * @version V1.0
 * @date    9-September-2020
 */

/* Includes ------------------------------------------------------------------*/
#include "rc_sensor.h"  /* 遥控器传感器头文件，包含 rc_sensor_t 结构体定义 */
#include "rp_math.h"    /* 数学库，包含 DeathZoom（死区）函数 */

/* External function declarations（在其他模块中实现的函数） */
extern void rc_sensor_init(rc_sensor_t *rc_sen);      /* 遥控器初始化 */
extern void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf);  /* 遥控器数据更新 */

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_check(rc_sensor_t *rc_sen);     /* 遥控器数据检查（内部函数） */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen); /* 遥控器心跳包（内部函数） */

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/**
 * @brief 遥控器 UART 驱动配置
 * 使用 UART3 接收 DBUS 协议数据
 * tx_byte 设为 NULL 表示只接收不发送
 */
drv_uart_t rc_sensor_driver = {
    .id = DRV_UART3,       /* UART3 外设 */
    .tx_byte = NULL,       /* 不使用发送功能 */
};

/**
 * @brief 遥控器信息结构体
 * 包含拨轮跳变判断阈值和离线检测参数
 */
rc_sensor_info_t rc_sensor_info = {
  /* 拨轮跳变判断阈值，对应数组内的名称的宏定义----------类似enum的0到3*/
    .tw_step_value[RC_TB_UP] = -600,   /* 推到顶部跳变阈值 */
    .tw_step_value[RC_TB_MU] = -200,   /* 上推一点跳变阈值 */
    .tw_step_value[RC_TB_DN] = +600,   /* 推到底部跳变阈值 */
    .tw_step_value[RC_TB_MD] = +200,   /* 下推一点跳变阈值 */
    .offline_max_cnt = 60,              /* 最大离线计数：60 次（约 600ms） */
};

/**
 * @brief 遥控器传感器全局实例
 * 所有模块通过此全局变量访问遥控器数据
 * 包含信息结构体、函数指针、工作状态等
 */
rc_sensor_t rc_sensor = {
    .info = &rc_sensor_info,            /* 遥控器信息结构体指针 */
    .init = rc_sensor_init,            /* 初始化函数指针 */
    .update = rc_sensor_update,        /* 数据更新函数指针 */
    .check = rc_sensor_check,          /* 数据检查函数指针 */
    .heart_beat = rc_sensor_heart_beat, /* 心跳包函数指针 */
    .work_state = DEV_OFFLINE,         /* 初始设备状态：离线 */
    .id = DEV_ID_RC,                   /* 设备 ID */
};

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  遥控器数据检查函数
 *         处理拨轮跳变检测、拨杆状态检测、数据范围校验
 * @param[in,out] rc_sen 遥控器传感器实例指针
 *
 * @note   拨轮跳变检测原理：
 *           1. 记录拨轮最大偏移量
 *           2. 拨轮回正后（值接近 0），根据最大偏移量判断跳变档位
 *           3. 跳变状态翻转（step[i] = !step[i]），实现按次触发
 *
 *         拨杆状态检测原理：
 *           1. 比较当前值和上次值
 *           2. 不同则更新状态为 up_R/mid_R/down_R
 *           3. 相同则保持为 keep_R
 *
 *         数据范围校验：
 *           - 摇杆通道有效范围：-660 ~ +660
 *           - 超出范围则清零所有数据（安全保护）
 */
static void rc_sensor_check(rc_sensor_t *rc_sen)
{
    /* ========== 拨轮跳变检测 ========== */
    static int16_t thumbwheel_record = 0;       /* 记录拨轮最大偏移量 */
    static uint8_t thumbwheel_last_step[4];     /* 记录上次各档位的跳变状态 */
    rc_sensor_info_t *rc_info = rc_sen->info;

    /*更新最大拨轮偏移量*/
    if ((abs(rc_info->thumbwheel.value_last) < abs(rc_info->thumbwheel.value)) &&
        (abs(thumbwheel_record) < abs(rc_info->thumbwheel.value)))
    {
        thumbwheel_record = rc_info->thumbwheel.value;   /* 刷新最高记录 */
    }

    /*拨轮回正后进行跳变判断--------条件：拨轮值回到中心区域（绝对值 <= 10），且有记录的最大偏移量*/
    if ((abs(rc_info->thumbwheel.value) <= 10) && (thumbwheel_record != 0))
    {
        /* 遍历 4 个跳变档位进行判断 */
        for (char i = 0; i < 4; i++)
        {
           /* 往下拨的情况：阈值是正数（下推到底 +600 / 下推微调 +200）*/
            if (rc_info->tw_step_value[i] > 0 && thumbwheel_record > 0)
            {
                if (thumbwheel_record >= rc_info->tw_step_value[i])
                {
                    rc_info->thumbwheel.step[i] = !rc_info->thumbwheel.step[i];  /* 翻转该档位状态(0变1或1变0) */
                    thumbwheel_record = 0;  /* 清零记录，防止重复触发 */
                }
            }
            /*  往上拨的情况：阈值是负数（上推到顶 -600 / 上推微调 -200） */
            if (rc_info->tw_step_value[i] < 0 && thumbwheel_record < 0)
            {
                if (thumbwheel_record <= rc_info->tw_step_value[i])
                {
                    rc_info->thumbwheel.step[i] = !rc_info->thumbwheel.step[i];  /* 状态翻转 */
                    thumbwheel_record = 0;  /* 清零记录，防止重复触发 */
                }
            }
        }
        thumbwheel_record = 0;  /* 清零记录 */
    }

    /**
     * 拨轮只要上面的 step 发生变化（不管 0 变 1 还是 1 变 0），
     * 就让 trigger 在当前帧变成 true（触发切模）
     */
    for (uint8_t i = 0; i < 4; i++)
    {
        if (thumbwheel_last_step[i] != rc_info->thumbwheel.step[i])
        {
            rc_info->thumbwheel.step_rising_trigger[i] = true;   /* 检测到跳变 */
        }
        else
        {
            rc_info->thumbwheel.step_rising_trigger[i] = false;  /* 无跳变 */
        }
        thumbwheel_last_step[i] = rc_info->thumbwheel.step[i];  /* 保存当前状态 */
    }

    /* 保存拨轮当前值为历史值 */
    rc_info->thumbwheel.value_last = rc_info->thumbwheel.value;

    
    
    
    /* ========== 拨杆状态切换检测 ========== */

    /**
     * 左拨杆（s1）状态检测
     * 拨杆值：1=上, 2=下, 3=中
     * 状态：up_R=上跳变, mid_R=中跳变, down_R=下跳变, keep_R=保持
     */
    if (rc_sen->info->s1.value != rc_sen->info->s1.value_last)
    {
        switch (rc_sen->info->s1.value)
        {
        case 1:
            rc_sen->info->s1.status = up_R;     /* 拨到上方 */
            break;
        case 3:
            rc_sen->info->s1.status = mid_R;    /* 拨到中间 */
            break;
        case 2:
            rc_sen->info->s1.status = down_R;   /* 拨到下方 */
            break;
        default:
            break;
        }
    }
    else
    {
        rc_sen->info->s1.status = keep_R;       /* 保持不变 */
    }

    /**
     * 右拨杆（s2）状态检测
     * 逻辑与左拨杆相同
     */
    if (rc_sen->info->s2.value != rc_sen->info->s2.value_last)
    {
        switch (rc_sen->info->s2.value)
        {
        case 1:
            rc_sen->info->s2.status = up_R;     /* 拨到上方 */
            break;
        case 3:
            rc_sen->info->s2.status = mid_R;    /* 拨到中间 */
            break;
        case 2:
            rc_sen->info->s2.status = down_R;   /* 拨到下方 */
            break;
        default:
            break;
        }
    }
    else
    {
        rc_sen->info->s2.status = keep_R;       /* 保持不变 */
    }

    /* 记住这次的值，供下一帧对比 */
    rc_sen->info->s1.value_last = rc_sen->info->s1.value;
    rc_sen->info->s2.value_last = rc_sen->info->s2.value;

    
    /* ========== 数据范围校验 ========== */
    if (abs(rc_info->ch0) > 660 ||
        abs(rc_info->ch1) > 660 ||
        abs(rc_info->ch2) > 660 ||
        abs(rc_info->ch3) > 660)
    {
        /* 数据异常：设置错误码，清零所有通道和开关数据 */
        rc_sen->errno = DEV_DATA_ERR;
        rc_info->ch0 = 0;
        rc_info->ch1 = 0;
        rc_info->ch2 = 0;
        rc_info->ch3 = 0;
        rc_info->s1.value = RC_SW_MID;      /* 拨杆恢复中间位置 */
        rc_info->s2.value = RC_SW_MID;
        rc_info->thumbwheel.value = 0;       /* 拨轮归零 */
        rc_info->thumbwheel.value_last = 0;
        rc_info->thumbwheel.step[RC_TB_UP] = 0;
        rc_info->thumbwheel.step[RC_TB_MU] = 0;
        rc_info->thumbwheel.step[RC_TB_MD] = 0;
        rc_info->thumbwheel.step[RC_TB_DN] = 0;
    }
    else
    {
        rc_sen->errno = NONE_ERR;  /* 数据正常 */
    }
}

/**
 * @brief  遥控器心跳包（离线检测）
 *         通过计数器检测遥控器是否离线
 * @param[in,out] rc_sen 遥控器传感器实例指针
 *
 * @note   每次调用 offline_cnt 加 1
 *         超过 offline_max_cnt（默认 60，约 600ms）则判定为离线
 *         每次成功接收数据会清零 offline_cnt
 *         遥控器离线后，系统应进入安全状态（停止电机等）
 */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *rc_info = rc_sen->info;

    rc_info->offline_cnt++;  /* 离线计数器递增 */

    if (rc_info->offline_cnt > rc_info->offline_max_cnt)
    {
        /* 超过最大离线次数，判定为离线 */
        rc_info->offline_cnt = rc_info->offline_max_cnt;  /* 防止溢出 */
        rc_sen->work_state = DEV_OFFLINE;
    }
    else
    {
        /* 未超限：如果之前离线则恢复为在线 */
        if (rc_sen->work_state == DEV_OFFLINE)
        {
            rc_sen->work_state = DEV_ONLINE;
        }
    }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  检查遥控器所有通道是否处于复位状态（中心位置）
 *         使用死区函数判断各通道值是否接近零
 * @return true: 所有通道都在死区内（复位状态）；false: 至少一个通道有输入
 *
 * @note   死区范围：50（即通道值在 -50 ~ +50 范围内视为零）
 *         用于检测遥控器是否处于中立状态
 */
bool RC_IsChannelReset(void)
{
    if ((DeathZoom(rc_sensor_info.ch0, 0, 50) == 0) &&  /* ch0 在死区内 */
        (DeathZoom(rc_sensor_info.ch1, 0, 50) == 0) &&  /* ch1 在死区内 */
        (DeathZoom(rc_sensor_info.ch2, 0, 50) == 0) &&  /* ch2 在死区内 */
        (DeathZoom(rc_sensor_info.ch3, 0, 50) == 0))    /* ch3 在死区内 */
    {
        return true;   /* 所有通道复位 */
    }
    return false;      /* 至少一个通道有输入 */
}

/**
 * @brief  遥控器数据复位函数
 *         将所有遥控器数据强制设置为默认值（安全状态）
 * @param[in,out] rc 遥控器传感器实例指针
 *
 * @note   复位内容包括：
 *           - 4 个摇杆通道：ch0-ch3 = 0
 *           - 2 个拨杆开关：s1, s2 = 中间位置
 *           - 鼠标：速度和位置全部归零
 *           - 键盘：所有按键值归零
 *           - 拨轮：值和跳变状态全部归零
 *         通常在遥控器断开或异常时调用，确保系统安全
 */
void RC_ResetData(rc_sensor_t *rc)
{
    /* 摇杆通道强制归零 */
    rc->info->ch0 = 0;
    rc->info->ch1 = 0;
    rc->info->ch2 = 0;
    rc->info->ch3 = 0;

    /* 拨杆开关强制设为中间位置 */
    rc->info->s1.value = RC_SW_MID;
    rc->info->s2.value = RC_SW_MID;

    /* 鼠标数据归零 */
    rc->info->mouse_vx = 0;       /* 鼠标 X 轴速度 */
    rc->info->mouse_vy = 0;       /* 鼠标 Y 轴速度 */
    rc->info->mouse_vz = 0;       /* 鼠标 Z 轴速度（滚轮） */
    rc->info->mouse_x = 0.f;      /* 鼠标 X 轴累计位移 */
    rc->info->mouse_y = 0.f;      /* 鼠标 Y 轴累计位移 */
    rc->info->mouse_z = 0.f;      /* 鼠标 Z 轴累计位移 */
    rc->info->mouse_btn_l.value = 0;  /* 鼠标左键 */
    rc->info->mouse_btn_r.value = 0;  /* 鼠标右键 */

    /* 键盘数据归零 */
    rc->info->key_v = 0;          /* 键盘按键值（位掩码） */
    rc->info->W.value = 0;        /* W 键 */
    rc->info->S.value = 0;        /* S 键 */
    rc->info->A.value = 0;        /* A 键 */
    rc->info->D.value = 0;        /* D 键 */
    rc->info->Shift.value = 0;    /* Shift 键 */
    rc->info->Ctrl.value = 0;     /* Ctrl 键 */
    rc->info->Q.value = 0;        /* Q 键 */
    rc->info->E.value = 0;        /* E 键 */
    rc->info->R.value = 0;        /* R 键 */
    rc->info->F.value = 0;        /* F 键 */
    rc->info->G.value = 0;        /* G 键 */
    rc->info->Z.value = 0;        /* Z 键 */
    rc->info->X.value = 0;        /* X 键 */
    rc->info->C.value = 0;        /* C 键 */
    rc->info->V.value = 0;        /* V 键 */
    rc->info->B.value = 0;        /* B 键 */

    /* 拨轮数据归零 */
    rc->info->thumbwheel.value = 0;        /* 拨轮当前值 */
    rc->info->thumbwheel.value_last = 0;   /* 拨轮上次值 */
    rc->info->thumbwheel.step[RC_TB_UP] = 0;  /* 顶部跳变状态 */
    rc->info->thumbwheel.step[RC_TB_MU] = 0;  /* 上推一点跳变状态 */
    rc->info->thumbwheel.step[RC_TB_MD] = 0;  /* 下推一点跳变状态 */
    rc->info->thumbwheel.step[RC_TB_DN] = 0;  /* 底部跳变状态 */
}
