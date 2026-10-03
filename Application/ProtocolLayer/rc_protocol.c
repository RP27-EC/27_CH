/**
 * @file    rc_protocol.c
 * @brief   遥控器/键鼠数据协议解析模块
 *
 * 本文件实现了遥控器（RC）和键鼠（BM）数据的解析、滤波和状态管理功能。
 * 支持 DJI 遥控器协议，包括通道数据解析、拨轮数据解析、开关状态解析，
 * 以及键盘按键的短按/长按状态机管理。还包含鼠标速度的滑动均值滤波算法。
 */

/* Includes ------------------------------------------------------------------*/
#include "rc_protocol.h"    /* 遥控器协议头文件 */
#include "rp_math.h"        /* 数学工具库 */
#include "rc_sensor.h"      /* 遥控器传感器驱动 */

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* 内部函数前向声明 */
void keyboard_cnt_max_set(rc_sensor_t *rc_sen);       /* 设置按键长按阈值 */
void keyboard_status_update(key_board_info_t *key);   /* 单个按键状态更新 */

extern uint32_t micros(void);  /* 微秒级计时函数（外部声明） */

/* 调试用时间戳变量 */
uint32_t tt1, tt2, ttp1;

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  遥控器传感器初始化函数
 * @param  rc_sen: 遥控器传感器结构体指针
 * @note   初始化遥控器为离线状态，重置数据，设置按键长按阈值，
 *         并根据设备ID判断初始化是否正确
 */
void rc_sensor_init(rc_sensor_t *rc_sen)
{
    /* 初始化为离线状态：将离线计数器设为最大值+1，确保初始状态为离线 */
    rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
    rc_sen->work_state = DEV_OFFLINE;   /* 设备状态标记为离线 */

    RC_ResetData(rc_sen);               /* 重置遥控器数据为默认值 */
    keyboard_cnt_max_set(rc_sen);       /* 设置各按键的长按时间阈值 */

    /* 校验设备ID是否正确 */
    if(rc_sen->id == DEV_ID_RC)
        rc_sen->errno = NONE_ERR;       /* 设备ID正确，无错误 */
    else
        rc_sen->errno = DEV_ID_ERR;     /* 设备ID错误 */
}

/**
 * @brief  按键长按时间阈值设置
 * @param  rc_sen: 遥控器传感器结构体指针
 * @note   为每个按键和鼠标按钮设置长按判定的最大计数值。
 *         当按键持续按下的时间超过 cnt_max 时，状态将从短按切换为长按。
 */
void keyboard_cnt_max_set(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *info = rc_sen->info;

    /* 鼠标左键和右键的长按阈值设置 */
    info->mouse_btn_l.cnt_max = MOUSE_BTN_L_CNT_MAX;  /* 鼠标左键长按阈值 */
    info->mouse_btn_r.cnt_max = MOUSE_BTN_R_CNT_MAX;  /* 鼠标右键长按阈值 */

    /* 键盘各按键的长按阈值设置 */
    info->Q.cnt_max = KEY_Q_CNT_MAX;
    info->W.cnt_max = KEY_W_CNT_MAX;
    info->E.cnt_max = KEY_E_CNT_MAX;
    info->R.cnt_max = KEY_R_CNT_MAX;
    info->A.cnt_max = KEY_A_CNT_MAX;
    info->S.cnt_max = KEY_S_CNT_MAX;
    info->D.cnt_max = KEY_D_CNT_MAX;
    info->F.cnt_max = KEY_F_CNT_MAX;
    info->G.cnt_max = KEY_G_CNT_MAX;
    info->Z.cnt_max = KEY_Z_CNT_MAX;
    info->X.cnt_max = KEY_X_CNT_MAX;
    info->C.cnt_max = KEY_C_CNT_MAX;
    info->V.cnt_max = KEY_V_CNT_MAX;
    info->B.cnt_max = KEY_B_CNT_MAX;
    info->Shift.cnt_max = KEY_SHIFT_CNT_MAX;
    info->Ctrl.cnt_max  = KEY_CTRL_CNT_MAX;
}

/**
 * @brief  鼠标速度数据滑动均值滤波更新
 * @param  rc_sen: 遥控器传感器结构体指针
 * @note   采用滑动窗口均值滤波算法，对鼠标X/Y轴的速度数据进行平滑处理。
 *         窗口大小为 REMOTE_SMOOTH_TIMES，每次更新时：
 *         1. 移除最旧的数据点
 *         2. 加入最新的数据点
 *         3. 重新计算均值作为滤波后的鼠标位移量
 */
void rc_interrupt_update(rc_sensor_t *rc_sen)
{
    /* 滑动均值滤波的缓冲区和索引（静态变量，保持状态） */
    static int16_t mouse_x[REMOTE_SMOOTH_TIMES], mouse_y[REMOTE_SMOOTH_TIMES];
    static int16_t index = 0;

    /* 索引回绕，实现环形缓冲区 */
    if(index == REMOTE_SMOOTH_TIMES)
    {
        index = 0;
    }

    /* 移除最旧的数据点对当前均值的贡献 */
    rc_sen->info->mouse_x -= (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
    rc_sen->info->mouse_y -= (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;

    /* 存入最新的原始速度数据 */
    mouse_x[index] = rc_sen->info->mouse_vx;
    mouse_y[index] = rc_sen->info->mouse_vy;

    /* 加入新数据点对当前均值的贡献 */
    rc_sen->info->mouse_x += (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
    rc_sen->info->mouse_y += (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;

    index++;  /* 索引前移 */
}

/**
 * @brief  遥控器数据解析协议（核心解析函数）
 * @param  rc_sen: 遥控器传感器结构体指针
 * @param  rxBuf:  接收缓冲区指针，包含原始字节数据
 * @note   按照 DJI 遥控器协议解析18字节数据帧：
 *         - 字节0~5:   4个通道数据（ch0~ch3），每个通道11位，中心值1024
 *         - 字节16~17: 拨轮数据，11位
 *         - 字节5:     两个开关状态（s1, s2），各2位
 *         - 字节6~11:  鼠标X/Y/Z轴速度数据，16位有符号
 *         - 字节12~13: 鼠标左键/右键状态
 *         - 字节14~15: 键盘按键位掩码，16位
 */
void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf)
{
    rc_sensor_info_t *rc_info = rc_sen->info;
    rc_info->offline_cnt = 0;   /* 收到数据，重置离线计数器 */

    /* ===== 遥控器通道数据解析 ===== */
    /* ch0: 字节0低8位 + 字节1低3位，11位数据，中心值1024，偏移后范围[-660, +660] */
    rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF;
    rc_info->ch0 -= 1024;  /* 减去中心值，转换为有符号偏移量 */

    /* ch1: 字节1高5位 + 字节2低5位 */
    rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF;
    rc_info->ch1 -= 1024;

    /* ch2: 字节2高2位 + 字节3 + 字节4低2位 */
    rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
    rc_info->ch2 -= 1024;

    /* ch3: 字节4高6位 + 字节5低5位 */
    rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF;
    rc_info->ch3 -= 1024;

    /* ===== 拨轮数据解析（11位） ===== */
    rc_info->thumbwheel.value = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
    rc_info->thumbwheel.value -= 1024;  /* 减去中心值 */

    /* ===== 开关状态解析 ===== */
    /* s1: 2位开关，位于字节5的第4~5位 */
    rc_info->s1.value = ((rxBuf[5] >> 4) & 0x000C) >> 2;
    /* s2: 2位开关，位于字节5的第6~7位 */
    rc_info->s2.value = (rxBuf[5] >> 4) & 0x0003;

    /* 遥控器限位归零处理：当ch3达到最小值-660时，将其置零 */
    if(rc_sensor.info->ch3 == -660)
    {
        rc_sensor.info->ch3 = 0;
    }

    /* ===== 键鼠数据解析 ===== */
    /* 鼠标速度数据：16位有符号整数 */
    rc_info->mouse_vx = rxBuf[6]  | (rxBuf[7 ] << 8);   /* 鼠标X轴速度 */
    rc_info->mouse_vy = rxBuf[8]  | (rxBuf[9 ] << 8);   /* 鼠标Y轴速度 */
    rc_info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8);   /* 鼠标Z轴（滚轮）速度 */

    /* 鼠标按键状态：最低位有效 */
    rc_info->mouse_btn_l.value = rxBuf[12] & 0x01;       /* 鼠标左键 */
    rc_info->mouse_btn_r.value = rxBuf[13] & 0x01;       /* 鼠标右键 */

    /* 键盘按键位掩码：16位，每一位对应一个按键 */
    rc_info->key_v = rxBuf[14] | (rxBuf[15] << 8);

    /* 从键盘位掩码中提取各按键状态（宏定义见头文件） */
    rc_info->W.value    = KEY_PRESSED_W;
    rc_info->S.value    = KEY_PRESSED_S;
    rc_info->A.value    = KEY_PRESSED_A;
    rc_info->D.value    = KEY_PRESSED_D;
    rc_info->Shift.value = KEY_PRESSED_SHIFT;
    rc_info->Ctrl.value  = KEY_PRESSED_CTRL;
    rc_info->Q.value    = KEY_PRESSED_Q;
    rc_info->E.value    = KEY_PRESSED_E;
    rc_info->R.value    = KEY_PRESSED_R;
    rc_info->F.value    = KEY_PRESSED_F;
    rc_info->G.value    = KEY_PRESSED_G;
    rc_info->Z.value    = KEY_PRESSED_Z;
    rc_info->X.value    = KEY_PRESSED_X;
    rc_info->C.value    = KEY_PRESSED_C;
    rc_info->V.value    = KEY_PRESSED_V;
    rc_info->B.value    = KEY_PRESSED_B;

    /* 再次确认离线计数器为0（已收到有效数据） */
    rc_info->offline_cnt = 0;

    /* 计算遥控器数据接收周期（微秒），用于调试和性能监测 */
    tt1 = tt2;              /* 保存上一次的时间戳 */
    tt2 = micros();         /* 获取当前微秒时间戳 */
    ttp1 = tt2 - tt1;       /* 计算两次接收之间的时间差 */
}

/**
 * @brief  批量更新所有按键状态
 * @param  info: 遥控器信息结构体指针
 * @note   依次调用 keyboard_status_update() 更新所有按键和鼠标按钮的状态机
 */
void keyboard_update(rc_sensor_info_t *info)
{
    /* 更新鼠标按钮状态 */
    keyboard_status_update(&info->mouse_btn_l);
    keyboard_status_update(&info->mouse_btn_r);

    /* 更新键盘各按键状态 */
    keyboard_status_update(&info->Q);
    keyboard_status_update(&info->W);
    keyboard_status_update(&info->E);
    keyboard_status_update(&info->R);
    keyboard_status_update(&info->A);
    keyboard_status_update(&info->S);
    keyboard_status_update(&info->D);
    keyboard_status_update(&info->F);
    keyboard_status_update(&info->G);
    keyboard_status_update(&info->Z);
    keyboard_status_update(&info->X);
    keyboard_status_update(&info->C);
    keyboard_status_update(&info->V);
    keyboard_status_update(&info->B);
    keyboard_status_update(&info->Shift);
    keyboard_status_update(&info->Ctrl);
}

/**
 * @brief  单个按键状态机更新
 * @param  key: 按键信息结构体指针
 * @note   按键状态转移流程：
 *         release（释放）-> release_to_press（刚按下）-> short_press（短按）
 *         -> long_press（长按）-> press_to_release（松开）
 *
 *         状态判定逻辑：
 *         - value == 0 且 cnt != 0：按键刚释放（press_to_release）
 *         - value == 0 且 cnt == 0：按键持续释放（release）
 *         - value == 1 且 cnt == 1：按键刚按下（release_to_press）
 *         - value == 1 且 cnt < cnt_max：按键短按中（short_press）
 *         - value == 1 且 cnt >= cnt_max：按键长按（long_press）
 */
void keyboard_status_update(key_board_info_t *key)
{
    key->last_status = key->status;  /* 保存上一次状态，用于边沿检测 */

    switch(key->value)
    {
        case 0:  /* 按键未按下 */
        {
            if(key->cnt != 0)
            {
                /* 之前有按下记录，现在松开：触发 press_to_release 状态 */
                key->status = press_to_release;
                key->cnt = 0;  /* 清零计数器 */
            }
            else
            {
                /* 持续未按下：保持 release 状态 */
                key->status = release;
                key->cnt = 0;
            }
            break;
        }
        case 1:  /* 按键按下中 */
        {
            key->cnt++;  /* 按下计数器递增 */

            if(key->cnt == 1)
            {
                /* 首次检测到按下：release_to_press 状态 */
                key->status = release_to_press;
            }
            else if(key->cnt >= key->cnt_max)
            {
                /* 按下时间超过阈值：进入 long_press 长按状态 */
                key->status = long_press;
                key->cnt = key->cnt_max;  /* 计数器钳位，防止溢出 */
            }
            else
            {
                /* 按下但未达长按阈值：保持 short_press 短按状态 */
                key->status = short_press;
            }
        }
    }
}
