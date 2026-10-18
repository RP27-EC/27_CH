/**
  * @file        rc_protocol.c
  * @author      RobotPilots
  * @Version     v1.1
  * @brief       DT7遥控器 & DR16接收机 协议解析模块
  * @details     本文件实现了大疆DT7遥控器通过DR16接收机以DBUS协议（UART5）传输
  *              的数据解析功能，包括：
  *              1. 四通道遥控杆数据解析（ch0~ch3，范围[-660, 660]）
  *              2. 拨轮数据解析（范围[-660, 660]）
  *              3. 两档拨杆开关S1/S2解析
  *              4. 鼠标三轴速度及左右键状态解析（键鼠模式）
  *              5. 键盘16个按键状态位解析
  *              6. 键盘按键状态机（短按/长按/释放等状态流转）
  *              7. 鼠标速度均值滤波平滑处理
  * @update
  *              v1.0(9-September-2020)
  *              v1.1(24-October-2021)
  *                  1.修改rc_potocol.c/.h->rc_protocol.c/.h
  */

/* Includes ------------------------------------------------------------------*/
#include "rc_protocol.h"
#include "rp_math.h"

#include "rc_sensor.h"


/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  设置所有按键的长按判定阈值
  */
void keyboard_cnt_max_set(rc_sensor_t *rc_sen);

/**
  * @brief  更新单个按键的状态机
  */
void keyboard_status_update(key_board_info_t *key);

extern uint32_t micros(void);  /* 获取微秒级系统时间（外部函数声明） */

/* 时间戳调试变量（用于测量中断间隔） */
uint32_t tt1, tt2, ttp1;

/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  遥控器传感器初始化
  * @details 将遥控器传感器设置为离线状态，清空数据，设置按键长按阈值，
  *          并验证设备ID是否正确。
  *
  * @param  rc_sen : 遥控器传感器对象指针
  * @retval None
  */
void rc_sensor_init(rc_sensor_t *rc_sen)
{
    /* 初始化为离线状态：离线计数设为最大值+1，确保立即被判定为离线 */
    rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
    rc_sen->work_state = DEV_OFFLINE;

    /* 清空遥控器数据 */
    RC_ResetData(rc_sen);

    /* 设置所有按键的长按判定阈值 */
    keyboard_cnt_max_set(rc_sen);

    /* 验证设备ID：只有DEV_ID_RC才合法 */
    if(rc_sen->id == DEV_ID_RC)
        rc_sen->errno = NONE_ERR;
    else
        rc_sen->errno = DEV_ID_ERR;
}

/**
  * @brief  设置所有按键的长按时间阈值
  * @details 为遥控器和键盘上的每个按键设置长按判定的最大计数值。
  *          当按键持续按下的时间超过cnt_max时，状态机判定为长按状态。
  *          鼠标按键与键盘按键使用不同的阈值宏定义。
  *
  * @param  rc_sen : 遥控器传感器对象指针
  * @retval None
  */
void keyboard_cnt_max_set(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *info = rc_sen->info;

    /* 鼠标左右键长按阈值 */
    info->mouse_btn_l.cnt_max = MOUSE_BTN_L_CNT_MAX;
    info->mouse_btn_r.cnt_max = MOUSE_BTN_R_CNT_MAX;

    /* 键盘各按键长按阈值 */
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
    info->Ctrl.cnt_max = KEY_CTRL_CNT_MAX;
}

/**
  * @brief  鼠标速度数据的滑动均值滤波
  * @details 使用滑动窗口均值滤波算法对鼠标X/Y轴速度进行平滑处理。
  *          滑动窗口大小为REMOTE_SMOOTH_TIMES。
  *          每次更新时移除窗口中最旧的采样值，加入最新的采样值，
  *          然后计算窗口内所有值的平均值作为滤波输出。
  *
  * @param  rc_sen : 遥控器传感器对象指针
  * @retval None
  *
  * @note   鼠标原始数据mouse_vx/mouse_vy是每帧增量值，
  *         滤波后mouse_x/mouse_y是平滑的累计位移值
  */
void rc_interrupt_update(rc_sensor_t *rc_sen)
{
    /* 滑动窗口缓冲区，存储最近REMOTE_SMOOTH_TIMES次的采样值 */
    static int16_t mouse_x[REMOTE_SMOOTH_TIMES], mouse_y[REMOTE_SMOOTH_TIMES];
    static int16_t index = 0;  /* 滑动窗口索引 */

    /* 窗口索引循环回绕 */
    if(index == REMOTE_SMOOTH_TIMES)
    {
        index = 0;
    }

    /* 步骤1：减去窗口中最旧的采样值（移出窗口） */
    rc_sen->info->mouse_x -= (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
    rc_sen->info->mouse_y -= (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;

    /* 步骤2：存入最新的采样值 */
    mouse_x[index] = rc_sen->info->mouse_vx;
    mouse_y[index] = rc_sen->info->mouse_vy;

    /* 步骤3：加上最新的采样值（加入窗口），完成滑动平均 */
    rc_sen->info->mouse_x += (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
    rc_sen->info->mouse_y += (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;

    index++;
}

/**
  * @brief  遥控器DBUS数据解析协议
  * @details 解析DR16接收机通过UART5（DBUS协议）发送的21字节数据包。
  *          数据包格式为位域压缩，需要通过位操作提取各通道值。
  *
  *          数据包字节布局（21字节）：
  *          Byte 0~5:   遥控器4通道（ch0~ch3）和拨杆开关（s1, s2）
  *          Byte 6~13:  鼠标数据（vx, vy, vz速度 + 左右键）
  *          Byte 14~15: 键盘按键位图
  *          Byte 16~17: 拨轮数据
  *
  * @param  rc_sen : 遥控器传感器对象指针
  * @param  rxBuf  : UART接收缓冲区，包含21字节DBUS原始数据
  * @retval None
  */
void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf)
{
    rc_sensor_info_t *rc_info = rc_sen->info;

    /* ====== 遥控器四通道数据解析 ====== */
    /* ch0: 水平摇杆 —— 字节0低3位 + 字节1高8位（共11位），减去1024零偏后范围[-660, 660] */
    rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF;
    rc_info->ch0 -= 1024;

    /* ch1: 垂直摇杆 —— 字节1低3位 + 字节2高5位（共11位） */
    rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF;
    rc_info->ch1 -= 1024;

    /* ch2: 垂直摇杆 —— 字节2低6位 + 字节3高2位 + 字节4高4位（共11位） */
    rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
    rc_info->ch2 -= 1024;

    /* ch3: 水平摇杆 —— 字节4低7位 + 字节5高4位（共11位） */
    rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF;
    rc_info->ch3 -= 1024;

    /* ====== 拨轮数据解析（Byte 16~17，共11位） ====== */
    rc_info->thumbwheel.value = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
    rc_info->thumbwheel.value -= 1024;

    /* 拨轮值异常保护：超过有效范围则清零（防止硬件抖动或故障产生误触） */
    if(abs(rc_info->thumbwheel.value) > 660)
    {
        rc_info->thumbwheel.value = 0;
    }

    /* ====== 拨杆开关S1/S2解析（Byte 5高4位，各占2位） ====== */
    rc_info->s1 = ((rxBuf[5] >> 4) & 0x000C) >> 2;  /* S1: 3档开关，值为1/2/3 */
    rc_info->s2 = (rxBuf[5] >> 4) & 0x0003;         /* S2: 3档开关，值为1/2/3 */

    /* ====== 鼠标数据解析（Byte 6~13） ====== */
    rc_info->mouse_vx = rxBuf[6]  | (rxBuf[7]  << 8);  /* 鼠标X轴速度（int16有符号） */
    rc_info->mouse_vy = rxBuf[8]  | (rxBuf[9]  << 8);  /* 鼠标Y轴速度（int16有符号） */
    rc_info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8);  /* 鼠标Z轴速度（滚轮，int16有符号） */
    rc_info->mouse_btn_l.value = rxBuf[12] & 0x01;     /* 鼠标左键状态：0=释放，1=按下 */
    rc_info->mouse_btn_r.value = rxBuf[13] & 0x01;     /* 鼠标右键状态：0=释放，1=按下 */

    /* ====== 键盘按键位图解析（Byte 14~15，共16位，每位对应一个按键） ====== */
    rc_info->key_v = rxBuf[14] | (rxBuf[15] << 8);

    /* 通过位掩码宏提取各按键的当前按压状态（0/1） */
    rc_info->W.value     = KEY_PRESSED_W;
    rc_info->S.value     = KEY_PRESSED_S;
    rc_info->A.value     = KEY_PRESSED_A;
    rc_info->D.value     = KEY_PRESSED_D;
    rc_info->Shift.value = KEY_PRESSED_SHIFT;
    rc_info->Ctrl.value  = KEY_PRESSED_CTRL;
    rc_info->Q.value     = KEY_PRESSED_Q;
    rc_info->E.value     = KEY_PRESSED_E;
    rc_info->R.value     = KEY_PRESSED_R;
    rc_info->F.value     = KEY_PRESSED_F;
    rc_info->G.value     = KEY_PRESSED_G;
    rc_info->Z.value     = KEY_PRESSED_Z;
    rc_info->X.value     = KEY_PRESSED_X;
    rc_info->C.value     = KEY_PRESSED_C;
    rc_info->V.value     = KEY_PRESSED_V;
    rc_info->B.value     = KEY_PRESSED_B;

    /* 记录中断时间戳（调试用，测量两帧遥控数据之间的时间间隔） */
    tt1 = tt2;
    tt2 = micros();
    ttp1 = tt2 - tt1;
}

/**
  * @brief  批量更新所有键盘按键的状态机
  * @details 遍历遥控器信息结构体中所有按键（鼠标键+键盘键），
  *          对每个按键调用keyboard_status_update()进行状态流转。
  *
  * @param  info : 遥控器信息结构体指针
  * @retval None
  */
void keyboard_update(rc_sensor_info_t *info)
{
    /* 更新鼠标左右键状态 */
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
  * @brief  单个键盘按键状态机更新
  * @details 实现按键的五状态流转检测，支持短按、长按、边沿触发等功能。
  *
  *          状态流转图：
  *          release（释放） -> release_to_press（按下瞬间） -> short_press（短按） -> long_press（长按） -> press_to_release（松开瞬间）
  *                 ^                                                                |
  *                 +----------------------------------------------------------------+
  *
  * @param  key : 按键信息结构体指针
  * @retval None
  */
void keyboard_status_update(key_board_info_t *key)
{
    /* 保存上一次的状态，用于边沿检测 */
    key->last_status = key->status;

    switch(key->value)
    {
        case 0:  /* 按键未按下 */
        {
            if(key->cnt != 0)
            {
                /* 之前有按下记录，现在松开 -> press_to_release（刚释放状态） */
                key->status = press_to_release;
                key->cnt = 0;
            }
            else
            {
                /* 一直未按下 -> release（释放状态） */
                key->status = release;
                key->cnt = 0;
            }
            break;
        }
        case 1:  /* 按键按下 */
        {
            key->cnt++;  /* 持续按下计数器递增 */

            if(key->cnt == 1)
            {
                /* 第一次检测到按下 -> release_to_press（刚按下状态，即边沿触发） */
                key->status = release_to_press;
            }
            else if(key->cnt >= key->cnt_max)
            {
                /* 持续按下时间超过阈值 -> long_press（长按状态） */
                key->status = long_press;
                key->cnt = key->cnt_max;  /* 计数器钳位，防止溢出 */
            }
            else
            {
                /* 按下但未超过阈值 -> short_press（短按状态） */
                key->status = short_press;
            }
        }
    }
}

/**
  * @brief  UART5接收中断回调函数 —— 遥控器数据处理入口
  * @details 当UART5收到完整的DBUS帧数据时，由底层串口驱动调用此函数。
  *          该函数负责：
  *          1. 重置离线计数器（收到数据即证明在线）
  *          2. 调用协议解析函数提取通道值和键鼠数据
  *          3. 执行按键状态机更新
  *          4. 执行鼠标速度滤波平滑
  *
  * @param  rxBuf : UART接收缓冲区，包含21字节DBUS原始数据
  * @retval None
  */
void USART5_rxDataHandler(uint8_t *rxBuf)
{
    uint32_t now_ms = HAL_GetTick(); /* 收帧时间，不使用控制任务的 2 ms 代替 DBUS 间隔。 */
    /* UART5为100kbps，18字节至少需要约2ms。小于2ms不可能是一整帧新数据，
     * 忽略重复提交，不能覆盖有效间隔/刷新心跳/重复累计鼠标目标。 */
    if (rc_sensor.info->frame_received &&
        now_ms - rc_sensor.info->frame_update_ms < 2U) {
        return;
    }
    /* 每收到一帧完整的DBUS数据就重置离线计数器（包括第一帧） */
    rc_sensor.info->offline_cnt = 0;

    /* 解析遥控器原始数据（通道值、拨杆、鼠标、键盘） */
    rc_sensor.update(&rc_sensor, rxBuf);

    /* 检查遥控器数据是否通过校验 */
    rc_sensor.check(&rc_sensor);

    /* 对鼠标速度进行滑动均值滤波 */
    rc_interrupt_update(&rc_sensor);

    /* 校验成功的相邻帧才有可用间隔；错误后下一帧重新建立时间基准。 */
    if (rc_sensor.errno == NONE_ERR) {
        rc_sensor.info->frame_interval_ms = rc_sensor.info->frame_received ?
            now_ms - rc_sensor.info->frame_update_ms : 0U;
        rc_sensor.info->frame_update_ms = now_ms;
        rc_sensor.info->frame_received = true;
    } else {
        rc_sensor.info->frame_interval_ms = 0U;
        rc_sensor.info->frame_received = false;
    }

    /* 帧序号自增：消费侧（云台鼠标瞄准）靠它做「一帧只累加一次」的闸门 */
    rc_sensor.info->frame_seq++;
}
