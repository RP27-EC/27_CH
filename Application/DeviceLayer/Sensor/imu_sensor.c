/**
 * @file    imu_sensor.c
 * @brief   IMU 传感器管理模块
 *
 * 本文件实现了 IMU 传感器的高层管理功能，包括：
 *   1. IMU 初始化（BMI088 驱动 + 姿态解算算法初始化）
 *   2. IMU 数据更新（读取原始数据 -> 坐标变换 -> 校准 -> 姿态解算）
 *   3. IMU 心跳包（离线检测）
 *   4. IMU 温度控制（PID 恒温控制，当前未启用）
 *   5. 陀螺仪零偏校准（静态校准 2000 次取平均）
 *
 * 支持两种姿态解算算法（通过 rp_config.h 宏选择）：
 *   - Mahony 互补滤波（IMU_USE_MAHONY == 1）
 *   - 扩展卡尔曼滤波 EKF（IMU_USE_EKF == 1）
 *
 * @note   本模块是 IMU 数据的统一出口，其他模块通过 imu_sensor 全局实例获取姿态数据
 * @version V1.0
 */

/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"     /* IMU 传感器头文件，包含 imu_sensor_t 结构体定义 */
#include "drv_gpio.h"       /* GPIO 驱动，用于控制 IMU 电源等 */
#include "drv_tick.h"       /* 系统 tick 驱动，提供 micros() 等时间函数 */

/* 条件编译：根据配置选择姿态解算算法 */
#if IMU_USE_EKF == 1
#include "bmi_EKF.h"        /* EKF 姿态解算头文件 */
#endif
#if IMU_USE_MAHONY == 1
#include "bmi.h"            /* Mahony 姿态解算头文件 */
#endif

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  IMU 初始化函数（内部函数）
 * @param  self IMU 传感器实例指针
 */
void imu_init(imu_sensor_t *self);

/**
 * @brief  IMU 心跳包函数（内部函数）
 * @param  heart 心跳状态结构体指针
 */
void imu_heart_beat(work_state_t *heart);

/**
 * @brief  IMU 数据更新函数（内部函数）
 * @param  self IMU 传感器实例指针
 */
void imu_update(imu_sensor_t *self);

/**
 * @brief  IMU 温度设置函数（内部函数）
 * @param  self IMU 传感器实例指针
 * @param  temp 目标温度
 */
void imu_set_temperature(imu_sensor_t *self, float temp);

#if IMU_USE_EKF == 1
/**
 * @brief  使用加速度计数据初始化四元数（内部函数）
 * @param  init_q4 输出的初始四元数 [q0, q1, q2, q3]
 */
static void InitQuaternion(float *init_q4);
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * @brief IMU 温度控制 PID 参数
 * kp=1000, ki=0, 输出限幅 4500（PWM 值）
 * 当前未启用温度控制功能
 */
pid_ctrl_t imu_temp_pid = {
    .kp = 1000.0f,         /* 比例系数 */
    .ki = 0.0f,            /* 积分系数 */
    .out_max = 4500,       /* 输出限幅（PWM 最大值） */
};

/**
 * @brief IMU 信息结构体
 * 包含零偏校准信息、初始化标志等
 */
imu_info_t imu_info =
{
    .offset_info.gx_offset = 0.f,  /* X 轴陀螺仪零偏 */
    .offset_info.gy_offset = 0.f,  /* Y 轴陀螺仪零偏 */
    .offset_info.gz_offset = 0.f,  /* Z 轴陀螺仪零偏 */
    .init_flag = 0,                /* 初始化标志：0=未初始化，1=已初始化 */
};

/* Exported variables --------------------------------------------------------*/
/**
 * @brief IMU 传感器全局实例
 * 所有模块通过此全局变量访问 IMU 数据
 * 包含驱动配置、工作状态、函数指针等完整信息
 */
imu_sensor_t imu_sensor = {

    .info = &imu_info,                      /* IMU 信息结构体指针 */
    .driver.tpye = DR_SPI2,                 /* 驱动类型：SPI2 通信 */
    .work_state.dev_state = DEV_OFFLINE,    /* 初始设备状态：离线 */
    .id = DEV_ID_IMU,                       /* 设备 ID */
    .work_state.cali_end = 0,               /* 校准结束标志：0=未结束 */
    .work_state.offline_max_cnt = 50,       /* 最大离线计数：50 次 */
    .work_state.err_cnt = 0,                /* 错误计数 */
    .temp_pid = &imu_temp_pid,              /* 温度控制 PID 指针 */

    .init = &imu_init,                      /* 初始化函数指针 */
    .update = &imu_update,                  /* 数据更新函数指针 */
    .heart_beat = &imu_heart_beat,          /* 心跳包函数指针 */
    .set_temperature = &imu_set_temperature, /* 温度设置函数指针 */
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @brief 陀螺仪数据临时存储数组（调试用） */
float imu_read[3];

/** @brief 初始化重试次数上限 */
uint8_t init_cnt = 200;

/**
 * @brief  IMU 初始化函数
 *         完成 BMI088 驱动初始化、姿态解算算法初始化、零偏校准准备
 * @param[in,out] self IMU 传感器实例指针
 *
 * @note   初始化流程：
 *           1. 调用 BMI088_init() 初始化传感器硬件
 *           2. 如果初始化失败，重试最多 init_cnt 次
 *           3. 如果仍然失败，触发系统复位（NVIC_SystemReset）
 *           4. 初始化成功后，设置设备状态为在线
 *           5. 根据配置初始化 Mahony 或 EKF 姿态解算算法
 *           6. 设置误差码为 IMU_DATA_CALI，开始零偏校准
 */
void imu_init(struct imu_struct *self)
{
    uint32_t tickstart = HAL_GetTick();  /* 记录初始化开始时间 */

    /* 设置设备状态为离线 */
    self->work_state.dev_state = DEV_OFFLINE;

    /* 调用 BMI088 驱动初始化函数 */
    self->work_state.init_code = BMI088_init();

    /* 初始化失败时重试 */
    while(self->work_state.init_code)
    {
        if (++self->work_state.err_cnt == init_cnt)
        {
            /* 重试次数超限：关闭中断并复位系统 */
            __set_FAULTMASK(1);      /* 关闭所有中断 */
            NVIC_SystemReset();      /* 系统复位 */
            break;
        }
        self->work_state.err_code = IMU_INIT_ERR;  /* 设置初始化错误码 */
        self->work_state.init_code = BMI088_init(); /* 重新尝试初始化 */
    }

    /* 初始化成功处理 */
    if (self->work_state.init_code == 0)
    {
        self->work_state.dev_state = DEV_ONLINE;  /* 设备状态设为在线 */
        self->info->init_flag = 1;                /* 标记初始化完成 */

#if IMU_USE_MAHONY == 1
        /* 初始化 Mahony 姿态解算：计算坐标变换矩阵 */
        transform_init(&gim_trans);
#endif

#if IMU_USE_EKF == 1
        /* 初始化 EKF 姿态解算：计算坐标变换矩阵 */
        transform_init(&EKFgim_trans);

        /* 使用加速度计数据初始化四元数（避免初始姿态误差） */
        float init_quaternion[4] = {0};
        InitQuaternion(init_quaternion);

        /* 初始化 EKF 滤波器
         * 参数：初始四元数, Q1=10, Q2=0.001, R=8000000, lambda=1
         */
        IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 8000000, 1);
#endif

        /**
         * 设置误差码为 IMU_DATA_CALI，启动零偏校准
         * 校准完成后会切换为 IMU_NONE_ERR
         */
        self->work_state.err_code = IMU_DATA_CALI;

        /* 清零零偏校准值 */
        imu_sensor.info->offset_info.gx_offset = 0.f;
        imu_sensor.info->offset_info.gy_offset = 0.f;
        imu_sensor.info->offset_info.gz_offset = 0.f;
    }
    else
    {
        /* 初始化失败：设置离线状态和错误码 */
        self->work_state.dev_state = DEV_OFFLINE;
        self->work_state.err_code = IMU_INIT_ERR;
        self->work_state.offline_cnt = self->work_state.offline_max_cnt;
        self->info->init_flag = 0;
    }
}

/**
 * @brief  IMU 失联检测（心跳包）
 *         通过计数器检测 IMU 是否离线
 * @param[in,out] heart 心跳状态结构体指针
 *
 * @note   每次调用 offline_cnt 加 1
 *         超过 offline_max_cnt 则判定为离线
 *         每次成功读取数据会清零 offline_cnt
 */
void imu_heart_beat(work_state_t *heart)
{
    heart->offline_cnt++;  /* 离线计数器递增 */

    if(heart->offline_cnt > heart->offline_max_cnt)
    {
        /* 超过最大离线次数，判定为离线 */
        heart->offline_cnt = heart->offline_max_cnt;  /* 防止溢出 */
        heart->dev_state = DEV_OFFLINE;
    }
    else
    {
        /* 未超限：如果之前离线则恢复为在线 */
        if (heart->dev_state == DEV_OFFLINE)
        {
            heart->dev_state = DEV_ONLINE;
        }
    }
}

/**
 * @brief  IMU 温度设置函数
 *         通过 PID 控制 IMU 加热电阻的 PWM 占空比
 * @param[in,out] self IMU 传感器实例指针
 * @param[in]     temp 目标温度（摄氏度）
 *
 * @note   当前功能已注释，未启用
 *         启用后可通过 PID 控制将 IMU 温度恒定在设定值
 *         有利于减少温度漂移对传感器精度的影响
 */
void imu_set_temperature(imu_sensor_t *self, float temp)
{
    /* 温度控制 PID 代码（当前已注释） */
}

/* ==================== 均值滤波器实例 ==================== */
/** @brief Pitch 轴角速度均值滤波器 */
ave_filter_t imu_pitch_dif_speed_ave_filter;
/** @brief Roll 轴角速度均值滤波器 */
ave_filter_t imu_roll_dif_speed_ave_filter;
/** @brief Yaw 轴角速度均值滤波器 */
ave_filter_t imu_yaw_dif_speed_ave_filter;

/* 临时变量 */
static float pitch, roll, yaw;             /* 欧拉角 */
static float gyrox, gyroy, gyroz;          /* 坐标变换后的角速度 */
static float accx, accy, accz;             /* 坐标变换后的加速度 */
static float gyrox_, gyroy_, gyroz_;       /* 低通滤波后的角速度 */
static float accx_, accy_, accz_;          /* 低通滤波后的加速度 */
static float gyro[3], accel[3], temp;      /* 原始数据临时存储 */
static int16_t imu_cnt = 0;               /* 校准计数器 */

#if IMU_USE_EKF == 1
/** @brief EKF 采样周期（秒） */
static float imu_dt;
/** @brief EKF 当前时间戳（微秒） */
static uint32_t imu_tick_now;
/** @brief EKF 上次时间戳（微秒） */
static uint32_t imu_tick_last;
static uint8_t imu_tick_initialized;
#endif

/**
 * @brief  IMU 数据更新函数（核心函数）
 *         完成完整的 IMU 数据处理流程
 * @param[in,out] imu_sen IMU 传感器实例指针
 *
 * @note   完整数据处理流程：
 *           1. 从 BMI088 读取原始角速度和加速度数据
 *           2. 坐标系变换（陀螺仪坐标系 -> 云台坐标系）
 *           3. 陀螺仪零偏校准（静态校准 2000 次）
 *           4. 低通滤波（去噪）
 *           5. 姿态解算（Mahony 或 EKF）
 *           6. 计算世界坐标系加速度
 *           7. 计算角速度（度/秒）并均值滤波
 *           8. 检测数据异常（全零判断）
 *           9. 记录温度
 */
void imu_update(imu_sensor_t *imu_sen)
{
    imu_info_t *imu_info = imu_sen->info;  /* 获取 IMU 信息结构体 */

    /* ========== 步骤1：从 BMI088 读取原始数据 ========== */
    BMI088_read(gyro, accel, &temp);  /* 读取角速度(rad/s)、加速度(m/s^2)、温度 */

    /* 保存原始数据到信息结构体 */
    imu_info->raw_info.acc_x = accel[0];
    imu_info->raw_info.acc_y = accel[1];
    imu_info->raw_info.acc_z = accel[2];
    imu_info->raw_info.gyro_x = gyro[0];
    imu_info->raw_info.gyro_y = gyro[1];
    imu_info->raw_info.gyro_z = gyro[2];

    /* ========== 步骤2：坐标系变换 ========== */
    /* 将陀螺仪坐标系变换为云台坐标系 */
    Vector_Transform(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],
                     &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);

    /* ========== 步骤3：陀螺仪零偏校准 ========== */
    if (imu_sen->work_state.err_code == IMU_DATA_CALI)
    {
        /**
         * 静态校准模式：累加 2000 次取平均
         * offset = -sum(gyro) / 2000，保存静态均值的负数。
         * 控制任务标称周期为 2ms，采集约需 4 秒以上，期间须保持静止。
         */
        if (imu_cnt < 2000)
        {
            imu_info->offset_info.gx_offset -= gyrox * 0.0005f;
            imu_info->offset_info.gy_offset -= gyroy * 0.0005f;
            imu_info->offset_info.gz_offset -= gyroz * 0.0005f;
            imu_cnt++;
        }
        else
        {
            /* 校准完成（2000 次） */
            imu_cnt = 0;

            /**
             * 零偏有效性检查：如果零偏过大（>0.005），认为校准失败，清零
             * 正常情况下零偏应该很小
             */
            if (abs(imu_info->offset_info.gx_offset) > 0.005f)
                imu_info->offset_info.gx_offset = 0;
            if (abs(imu_info->offset_info.gy_offset) > 0.005f)
                imu_info->offset_info.gy_offset = 0;
            if (abs(imu_info->offset_info.gz_offset) > 0.005f)
                imu_info->offset_info.gz_offset = 0;

#if IMU_USE_EKF == 1
            /* 使用最终通过检查的 offset，同步转换为补偿后的残余零偏。
             * 此前 EKF 已运行：不可仅修改输入而保留原来的总零偏状态。 */
            IMU_QuaternionEKF_ShiftGyroBias(imu_info->offset_info.gx_offset,
                                           imu_info->offset_info.gy_offset);
#endif
            imu_sen->work_state.err_code = IMU_NONE_ERR;
            imu_sen->work_state.cali_end = 1;  /* 标记校准结束 */
        }
    }
    /* 独立于上面的校准分支：完成当帧即应用，和 EKF 状态平移同帧生效。 */
    if (imu_sen->work_state.err_code == IMU_NONE_ERR)
    {
        /* 两种算法都先扣除三轴静态零偏；EKF 再估计 X/Y 残余零偏。 */
        gyrox += imu_info->offset_info.gx_offset;
        gyroy += imu_info->offset_info.gy_offset;
        gyroz += imu_info->offset_info.gz_offset;
    }

    /* ========== 步骤4：原始数据低通滤波 ========== */
    /**
     * 低通滤波器：y(n) = alpha * x(n) + (1-alpha) * y(n-1)
     * alpha 越大，滤波越弱（响应快但噪声大）
     * alpha 越小，滤波越强（响应慢但数据平滑）
     */
    gyrox_ = Lowpass(gyrox_, gyrox, 1);    /* 角速度 X：alpha=1（不过滤） */
    gyroy_ = Lowpass(gyroy_, gyroy, 1);    /* 角速度 Y：alpha=1（不过滤） */
    gyroz_ = Lowpass(gyroz_, gyroz, 1);    /* 角速度 Z：alpha=1（不过滤） */
    accx_ = Lowpass(accx_, accx, 0.8);     /* 加速度 X：alpha=0.8（轻度滤波） */
    accy_ = Lowpass(accy_, accy, 0.2);     /* 加速度 Y：alpha=0.2（强滤波） */
    accz_ = Lowpass(accz_, accz, 0.2);     /* 加速度 Z：alpha=0.2（强滤波） */

    /* ========== 步骤5：姿态解算 ========== */
#if IMU_USE_MAHONY == 1
    /* Mahony 互补滤波算法：融合角速度和加速度计算欧拉角 */
    BMI_Get_EulerAngle(&imu_info->base_info.pitch, &imu_info->base_info.roll, &imu_info->base_info.yaw,
                       &gyrox_, &gyroy_, &gyroz_,
                       &accx_, &accy_, &accz_);
#endif

#if IMU_USE_EKF == 1
    /**
     * 扩展卡尔曼滤波算法
     * 需要计算精确的采样周期 dt
     */
    /* 计算采样时间 */
    imu_tick_now = micros();  /* 获取当前微秒时间戳 */

    if(!imu_tick_initialized)
    {
        /* 第一次调用使用控制任务的标称周期 2ms。 */
        imu_tick_last = imu_tick_now - 2000u;
        imu_tick_initialized = 1;
    }

    /* 计算采样周期（秒） */
    imu_dt = (imu_tick_now - imu_tick_last) * 0.000001f;  /* us -> s */
    imu_tick_last = imu_tick_now;

    /* 防止定时器异常导致 dt 过大 */
    /* uint32_t 差值支持正常回绕；重复或长间隔不伪造积分时间。 */
    if (imu_dt > 0.0f && imu_dt <= 0.01f)
        IMU_QuaternionEKF_Update(gyrox, gyroy, gyroz, accx, accy, accz, imu_dt);

    /* 从 EKF 结构体中提取结果 */
    imu_info->base_info.yaw = QEKF_INS.Yaw;
    imu_info->base_info.pitch = QEKF_INS.Pitch;
    imu_info->base_info.roll = QEKF_INS.Roll;
    imu_info->base_info.yaw_total_angle = QEKF_INS.YawTotalAngle;
#endif

    /* ========== 步骤6：获取世界坐标系加速度 ========== */
    pitch = imu_info->base_info.pitch;
    roll = imu_info->base_info.roll;
    yaw = imu_info->base_info.yaw;

    /* 将体坐标系加速度转换为世界坐标系加速度 */
    BMI_Get_Acceleration(pitch, roll, yaw,
                         accx_, accy_, accz_,
                         &imu_info->base_info.accx, &imu_info->base_info.accy, &imu_info->base_info.accz);

    /* ========== 步骤7：计算角速度（度/秒）并均值滤波 ========== */
    /**
     * 角速度单位转换：rad/s -> deg/s（除以 0.017453 = 乘以 57.2958）
     * 然后进行均值滤波（窗口大小为 3）
     */
    float real_gyrox = gyrox_;
    float real_gyroy = gyroy_;
#if IMU_USE_EKF == 1
    real_gyrox -= QEKF_INS.GyroBias[0];
    real_gyroy -= QEKF_INS.GyroBias[1];
#endif
    /* Pitch 轴角速度 */
    imu_info->base_info.rate_pitch = real_gyroy / 0.0174532925f;
    imu_info->base_info.ave_rate_pitch = ave_fil_update(&imu_pitch_dif_speed_ave_filter, imu_info->base_info.rate_pitch, 3);

    /* Roll 轴角速度 */
    imu_info->base_info.rate_roll = real_gyrox / 0.0174532925f;
    imu_info->base_info.ave_rate_roll = ave_fil_update(&imu_roll_dif_speed_ave_filter, imu_info->base_info.rate_roll, 3);

    /* Yaw 轴角速度 */
    imu_info->base_info.rate_yaw = gyroz_ / 0.0174532925f;
    imu_info->base_info.ave_rate_yaw = ave_fil_update(&imu_yaw_dif_speed_ave_filter, imu_info->base_info.rate_yaw, 3);

    /* 清零离线计数器（表示数据正常接收） */
    imu_sen->work_state.offline_cnt = 0;

    /* ========== 步骤8：IMU 数据异常检测 ========== */
    /**
     * 检测 IMU 数据是否全为零（可能表示传感器故障或通信中断）
     * 连续 100 次全零则判定为数据错误
     */
    if ((accel[0] == 0) && (accel[1] == 0) && (accel[2] == 0)
     && (gyro[0] == 0) && (gyro[1] == 0) && (gyro[2] == 0))
    {
        if(++imu_sen->work_state.err_cnt >= 100)
        {
            imu_sen->work_state.dev_state = DEV_OFFLINE;       /* 设备离线 */
            imu_sen->work_state.err_code = IMU_DATA_ERR;       /* 数据错误 */
            imu_sen->work_state.offline_cnt = imu_sen->work_state.offline_max_cnt;
            imu_sen->work_state.err_cnt = 100;                 /* 防止溢出 */
        }
    }

    /* ========== 步骤9：记录温度 ========== */
    imu_info->base_info.temperature = temp;
}

#if IMU_USE_EKF == 1
/**
 * @brief  使用加速度计数据初始化四元数
 *         通过读取 100 次加速度计数据取平均，计算初始 Roll 和 Pitch
 * @param[out] init_q4 输出的初始四元数 [q0, q1, q2, q3]
 *
 * @note   算法原理：
 *           1. 读取 100 次加速度计数据取平均（减少噪声）
 *           2. 从加速度计数据计算初始 pitch 和 roll
 *           3. yaw 初始化为 0（无法通过加速度计确定）
 *           4. 将欧拉角转换为四元数
 *         这样可以避免初始姿态估计误差导致的收敛问题
 */
static void InitQuaternion(float *init_q4)
{
    float acc_sum[3] = {0};        /* 加速度累加和 */
    float gyro_init[3], acc_init[3];  /* 临时读取缓冲区 */

    /* 读取 100 次加速度计数据，累加求和 */
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_read(gyro_init, acc_init, &temp);  /* 读取一次 IMU 数据 */
        acc_sum[0] += acc_init[0];  /* X 轴累加 */
        acc_sum[1] += acc_init[1];  /* Y 轴累加 */
        acc_sum[2] += acc_init[2];  /* Z 轴累加 */
        delay_ms(1);  /* 等待 1ms */
    }

    /* 计算平均值 */
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] = acc_sum[i] / 100;

    /* 初始化和运行阶段必须使用同一个安装坐标系。 */
    float gx, gy, gz, ax, ay, az;
    Vector_Transform(0, 0, 0, acc_init[0], acc_init[1], acc_init[2],
                     &gx, &gy, &gz, &ax, &ay, &az);
    acc_init[0] = ax;
    acc_init[1] = ay;
    acc_init[2] = az;
    float norm_sq = ax * ax + ay * ay + az * az;
    if (!isfinite(norm_sq) || norm_sq < 1e-6f)
    {
        init_q4[0] = 1.0f;
        init_q4[1] = init_q4[2] = init_q4[3] = 0.0f;
        return;
    }

    /* 从加速度计数据计算初始欧拉角 */
    float pitch, roll, temp;
    arm_sqrt_f32(acc_init[1] * acc_init[1] + acc_init[2] * acc_init[2], &temp);  /* sqrt(ay^2 + az^2) */
    arm_atan2_f32(-acc_init[0], temp, &pitch);   /* pitch = atan2(-ax, sqrt(ay^2+az^2)) */
    arm_atan2_f32(acc_init[1], acc_init[2], &roll);  /* roll = atan2(ay, az) */

    /* 将欧拉角转换为四元数 */
    float half_pitch = pitch / 2.0f;
    float half_roll = roll / 2.0f;

    /* 四元数计算公式（ZYX 欧拉角 -> 四元数） */
    init_q4[0] = arm_cos_f32(half_pitch) * arm_cos_f32(half_roll);  /* q0 = cos(p/2)*cos(r/2) */
    init_q4[1] = arm_cos_f32(half_pitch) * arm_sin_f32(half_roll);  /* q1: roll */
    init_q4[2] = arm_sin_f32(half_pitch) * arm_cos_f32(half_roll);  /* q2: pitch */
    init_q4[3] = -arm_sin_f32(half_pitch) * arm_sin_f32(half_roll); /* q3: yaw=0 */
}

#endif  /* IMU_USE_EKF */
