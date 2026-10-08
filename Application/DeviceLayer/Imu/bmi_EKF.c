/**
 ******************************************************************************
 * @file    bmi_EKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   基于扩展卡尔曼滤波（EKF）的 IMU 姿态解算
 *
 * 本文件实现了基于四元数扩展卡尔曼滤波器（Quaternion EKF）的姿态估计算法。
 * 相比 Mahony 互补滤波，EKF 具有更好的噪声抑制能力和自适应性。
 *
 * 主要功能：
 *   1. 四元数状态的卡尔曼预测和更新
 *   2. 陀螺仪零偏在线估计
 *   3. 卡方检验（Chi-square test）判断量测更新是否可靠
 *   4. 自适应卡尔曼增益调整
 *   5. 渐消因子防止零偏协方差过度收敛
 *   6. 坐标系变换（陀螺仪坐标系 -> 云台坐标系）
 *
 * 算法特点：
 *   - 状态向量：6 维 [q0, q1, q2, q3, gyro_bias_x, gyro_bias_y]
 *   - 量测向量：3 维 [accel_x, accel_y, accel_z]（归一化重力方向）
 *   - 过程噪声 Q 分为四元数噪声和零偏噪声两部分
 *   - 观测噪声 R 自适应调整
 *
 * @attention
 * 一阶低通滤波器传递函数：
 *     1
 *   ------
 *   as + 1
 ******************************************************************************
 */
#include "bmi_EKF.h"     /* EKF 姿态解算头文件 */
#include "rp_config.h"   /* 配置文件，包含 IMU_USE_EKF 宏定义 */

#if IMU_USE_EKF==1  /* 条件编译：仅在使用 EKF 算法时编译 */

/** @brief 标准重力加速度 (m/s^2) */
#define GRAVITY_EARTH  (9.80665f)

/* ==================== 矩阵实例定义 ==================== */
arm_matrix_instance_f32 EKFTrans;  /* EKF 用坐标变换矩阵（3x3） */
arm_matrix_instance_f32 EKFSrc;    /* EKF 用源向量（1x3） */
arm_matrix_instance_f32 EKFDst;    /* EKF 用目标向量（1x3） */

/**
 * @brief EKF 坐标变换参数
 * 与 bmi.c 中的 gim_trans 类似，用于将陀螺仪坐标变换为云台坐标
 * arz=90 度表示 IMU 安装时绕 Z 轴旋转了 90 度
 */
gimbal_transform_t EKFgim_trans = {
    /* 与 bmi.c gim_trans 保持一致：本车 arz=0（模板 90 与安装不符，见 bmi.c 注释） */
    .arz = 0.0f,    /* Z 轴旋转角度：0度 */
    .ary = 0.0f,    /* Y 轴旋转角度：0度 */
    .arx = 0.0f,    /* X 轴旋转角度：0度 */
    .trans = {0.0f}, /* 变换矩阵数组 */
};

/** @brief EKF 姿态解算核心结构体实例 */
QEKF_INS_t QEKF_INS;

/* ==================== EKF 矩阵初始化数据 ==================== */

/**
 * @brief 状态转移矩阵 F（6x6 单位矩阵初始值）
 * 状态向量：[q0, q1, q2, q3, gyro_bias_x, gyro_bias_y]
 * F 矩阵在每次预测步骤中会根据当前角速度动态更新
 */
const float IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

/**
 * @brief 协方差矩阵 P（6x6 初始值）
 * 对角线元素表示各状态的初始不确定性
 * 四元数部分初始不确定性设为 100000（大值，快速收敛）
 * 零偏部分初始不确定性设为 100（较小值）
 */
float IMU_QuaternionEKF_P[36] = {
    100000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 100000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 100000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 100000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 100, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 100
};

/** @brief 卡尔曼增益矩阵 K（6x3） */
float IMU_QuaternionEKF_K[18];

/** @brief 观测矩阵 H（3x6） */
float IMU_QuaternionEKF_H[18];

/* 前向声明：自定义卡尔曼滤波回调函数 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief  基于扩展卡尔曼滤波的姿态解算初始化
 * @param[in] init_quaternion  初始四元数 [q0, q1, q2, q3]
 * @param[in] process_noise1   四元数过程噪声协方差
 *                              越小 -> 解算数据越平滑
 *                              越大 -> 系统对快速变化反应越快
 *                              推荐值：10
 * @param[in] process_noise2   陀螺仪零偏估计过程噪声协方差
 *                              越小 -> 零偏估计越慢但更稳定
 *                              推荐值：0.001
 * @param[in] measure_noise    加速度计测量噪声协方差
 *                              越小 -> 越信任加速度计
 *                              越大 -> 更多依赖陀螺仪积分
 *                              推荐值：1000000
 * @param[in] lambda           渐消因子（0, 1]
 *                              用于防止陀螺仪零偏估计协方差过度收敛
 *                              推荐值：0.9996
 *                              设为 1 表示不使用渐消
 */
void IMU_QuaternionEKF_Init(float* init_quaternion, float process_noise1,
                            float process_noise2, float measure_noise, float lambda)
{
    /* 初始化 EKF 状态标志 */
    QEKF_INS.Initialized = 1;

    /* 设置噪声参数 */
    QEKF_INS.Q1 = process_noise1;       /* 四元数过程噪声 */
    QEKF_INS.Q2 = process_noise2;       /* 零偏过程噪声 */
    QEKF_INS.R = measure_noise;         /* 观测噪声 */

    /* 卡方检验参数 */
    QEKF_INS.ChiSquareTestThreshold = 3.5e-8;  /* 卡方检验阈值 */
    QEKF_INS.ConvergeFlag = 0;           /* 收敛标志：0=未收敛，1=已收敛 */
    QEKF_INS.ErrorCount = 0;             /* 连续未通过卡方检验的次数 */

    QEKF_INS.UpdateCount = 0;            /* 更新计数器 */

    /* 渐消因子限幅：最大为 1（不使用渐消） */
    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;

    /* 初始化卡尔曼滤波器：6维状态，0维控制输入，3维量测 */
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);

    /* 初始化卡方值矩阵（1x1） */
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);

    /* 设置初始姿态（四元数） */
    for(int i = 0; i < 4; i++)
    {
        QEKF_INS.IMU_QuaternionEKF.xhat_data[i] = init_quaternion[i];
    }

    /**
     * 注册自定义回调函数，用于扩展卡尔曼滤波器的标准流程
     * User_Func0_f: 观测环节（在量测更新前调用）
     * User_Func1_f: F 矩阵线性化 + P 矩阵渐消（在预测步骤中调用）
     * User_Func2_f: 设置观测矩阵 H（在量测更新前调用）
     * User_Func3_f: 后验估计更新（替换标准的 xhatUpdate）
     */
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    /**
     * 设置跳过标志：用自定义函数替换标准卡尔曼滤波步骤
     * SkipEq3: 跳过标准的 K = P*H'*(H*P*H'+R)^-1 计算
     * SkipEq4: 跳过标准的 xhat = xhat' + K*(z-H*xhat') 更新
     * 这两个步骤由 User_Func3_f（IMU_QuaternionEKF_xhatUpdate）替代
     */
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    /* 复制初始矩阵数据到卡尔曼滤波器内部存储 */
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief  基于扩展卡尔曼滤波的四元数更新（核心函数）
 * @param[in] gx, gy, gz  陀螺仪角速度数据（rad/s）
 * @param[in] ax, ay, az  加速度计数据（m/s^2）
 * @param[in] dt          数据更新周期（秒）
 *
 * @note   算法流程：
 *           1. 预测步骤：利用角速度更新状态转移矩阵 F，预测四元数
 *           2. 更新 F 矩阵右上角 4x2 分块（零偏对四元数的影响）
 *           3. 归一化加速度计数据作为量测值
 *           4. 判断加速度是否在合理范围内（稳定标志）
 *           5. 设置过程噪声 Q 和观测噪声 R
 *           6. 调用卡尔曼滤波更新函数
 *           7. 提取融合后的四元数和零偏
 *           8. 四元数转欧拉角
 *           9. 计算 Yaw 总角度（处理 360 度跳变）
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz,
                              float ax, float ay, float az, float dt)
{
    /* 临时变量：0.5 * (omega - bias) * dt，用于状态转移矩阵 */
    static float halfgxdt, halfgydt, halfgzdt;
    /* 加速度计归一化因子的倒数 */
    static float accelInvNorm;

    /**
     * F 矩阵结构（6x6）：
     * 左上角 4x4：四元数状态转移（与角速度相关）
     * 右上角 4x2：零偏对四元数的影响
     * 左下角 2x4：零（假设四元数不影响零偏）
     * 右下角 2x2：零偏状态转移（单位阵）
     *
     * 标记 * 的元素需要在每次预测时动态更新
     *  0      1*     2*     3*     4     5
     *  6*     7      8*     9*    10    11
     * 12*    13*    14     15*    16    17
     * 18*    19*    20*    21     22    23
     * 24     25     26     27     28    29
     * 30     31     32     33     34    35
     */

    /* 保存采样周期 */
    QEKF_INS.dt = dt;

    /**
     * 计算去偏后的角速度
     * omega_corrected = omega_measured - bias
     * 这是 EKF 的关键：通过估计零偏来修正角速度
     */
    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    /* ========== 设置状态转移矩阵 F ========== */
    /* 预计算 0.5 * omega * dt */
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    /**
     * 恢复 F 矩阵为单位阵（清除上一轮线性化的结果）
     * 因为预测步骤中 F 的右上角 4x2 部分是零矩阵
     */
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    /**
     * 设置 F 矩阵左上角 4x4 部分
     * 这是四元数微分方程的线性化结果
     * dq/dt = 0.5 * Omega(omega) * q
     */
    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    /* 保存加速度计原始数据 */
    QEKF_INS.Accel[0] = ax;
    QEKF_INS.Accel[1] = ay;
    QEKF_INS.Accel[2] = az;

    /* ========== 设置量测值 z ========== */
    /* 计算加速度计数据的模长（归一化因子） */
    arm_sqrt_f32(QEKF_INS.Accel[0] * QEKF_INS.Accel[0]
               + QEKF_INS.Accel[1] * QEKF_INS.Accel[1]
               + QEKF_INS.Accel[2] * QEKF_INS.Accel[2],
               &QEKF_INS.accl_norm);

    accelInvNorm = 1.0f / QEKF_INS.accl_norm;  /* 归一化因子的倒数 */

    /* 归一化加速度计数据，作为量测向量（单位重力方向） */
    for (uint8_t i = 0; i < 3; ++i)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm;
    }

    /* 计算角速度模长，用于判断运动状态 */
    arm_sqrt_f32(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0]
               + QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1]
               + QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2],
               &QEKF_INS.gyro_norm);

    /**
     * 判断载体是否处于稳定状态
     * 判断条件：加速度模长在 (9.8 - 5.5) 到 (9.8 + 5.5) 范围内
     * 即加速度接近重力加速度时认为稳定
     * StableFlag 在后续的卡方检验中用于决定是否更新
     */
    if (QEKF_INS.accl_norm > 9.8f - 5.5f && QEKF_INS.accl_norm < 9.8f + 5.5f)
    {
        QEKF_INS.StableFlag = 1;  /* 稳定：加速度可用于修正角速度 */
    }
    else
    {
        QEKF_INS.StableFlag = 0;  /* 不稳定：存在运动加速度，不可信 */
    }

    /* ========== 设置过程噪声 Q 和观测噪声 R ========== */
    /* Q 矩阵为对角阵，四元数部分噪声 Q1，零偏部分噪声 Q2 */
    QEKF_INS.IMU_QuaternionEKF.Q_data[0]  = QEKF_INS.Q1 * QEKF_INS.dt;  /* q0 噪声 */
    QEKF_INS.IMU_QuaternionEKF.Q_data[7]  = QEKF_INS.Q1 * QEKF_INS.dt;  /* q1 噪声 */
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;  /* q2 噪声 */
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;  /* q3 噪声 */
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;  /* gyro_bias_x 噪声 */
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;  /* gyro_bias_y 噪声 */

    /* R 矩阵为 3x3 对角阵 */
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;  /* x 轴观测噪声 */
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;  /* y 轴观测噪声 */
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;  /* z 轴观测噪声 */

    /**
     * 调用卡尔曼滤波器更新函数
     * 内部依次执行：
     *   1. 预测步骤（F 矩阵线性化 + 状态预测 + 协方差预测）
     *   2. 调用 User_Func0_f（观测环节）
     *   3. 调用 User_Func2_f（设置 H 矩阵）
     *   4. 调用 User_Func3_f（后验估计更新，含卡方检验）
     */
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    /* ========== 提取融合后的结果 ========== */
    /* 四元数 */
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];

    /* 陀螺仪零偏估计值 */
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0;  /* Z 轴零偏不估计（Z 轴通常通天，无法观测 yaw 漂移） */

    /* ========== 四元数转欧拉角 ========== */
    /* Yaw: atan2(2*(q0*q3 + q1*q2), 2*(q0^2 + q1^2) - 1) */
    arm_atan2_f32(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
                  2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f,
                  &QEKF_INS.Yaw);

    /* Roll: atan2(2*(q0*q1 + q2*q3), 2*(q0^2 + q3^2) - 1) */
    arm_atan2_f32(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
                  2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f,
                  &QEKF_INS.Roll);

    /* Pitch: 使用 atan2 避免万向节锁 */
    float sintemp, costemp;
    sintemp = -2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2]);
    arm_sqrt_f32(1 - sintemp*sintemp, &costemp);
    arm_atan2_f32(sintemp, costemp, &QEKF_INS.Pitch);

    /* 弧度转角度 */
    QEKF_INS.Yaw   *= 57.295779513f;
    QEKF_INS.Roll  *= 57.295779513f;
    QEKF_INS.Pitch *= 57.295779513f;

    /**
     * 计算 Yaw 总角度（累积角度）
     * 处理 Yaw 角度在 -180~+180 度之间的跳变
     * 通过记录圈数实现连续角度输出（如小陀螺功能需要）
     */
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f)
    {
        QEKF_INS.YawRoundCount--;  /* 从正到负跨越，圈数减 1 */
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f)
    {
        QEKF_INS.YawRoundCount++;  /* 从负到正跨越，圈数加 1 */
    }

    /* 总角度 = 圈数 * 360 + 当前角度 */
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;  /* 保存当前角度用于下次比较 */

    QEKF_INS.UpdateCount++;  /* 更新计数器（可用于初始化低通滤波等） */
}

/**
 * @brief  更新线性化后的状态转移矩阵 F 右上角的 4x2 分块矩阵
 *         并对零偏协方差进行渐消和限幅处理
 * @param[in,out] kf  卡尔曼滤波器实例指针
 *
 * @note   此函数作为 User_Func1_f 在卡尔曼预测步骤中被调用
 *         功能：
 *           1. 四元数归一化（防止数值漂移）
 *           2. 设置 F 矩阵右上角 4x2 分块（零偏对四元数的影响）
 *           3. 零偏协方差渐消（防止过度收敛）
 *           4. 零偏协方差限幅（防止发散）
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    /* 四元数归一化（将预测后的四元数规范化为单位四元数） */
    q0 = kf->xhatminus_data[0];
    arm_quaternion_normalize_f32(kf->xhatminus_data, kf->xhatminus_data, 1);

    /* 获取归一化后的四元数 */
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    /**
     * 设置 F 矩阵右上角 4x2 分块矩阵
     * 该分块表示零偏变化对四元数的影响
     *
     * F 矩阵结构：
     *  0     1     2     3     4*     5*
     *  6     7     8     9    10*    11*
     * 12    13    14    15    16*    17*
     * 18    19    20    21    22*    23*
     * 24    25    26    27    28    29
     * 30    31    32    33    34    35
     */
    kf->F_data[4]  =  q1 * QEKF_INS.dt / 2;    /* dq0/dbias_x */
    kf->F_data[5]  =  q2 * QEKF_INS.dt / 2;    /* dq0/dbias_y */

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;    /* dq1/dbias_x */
    kf->F_data[11] =  q3 * QEKF_INS.dt / 2;    /* dq1/dbias_y */

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;    /* dq2/dbias_x */
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;    /* dq2/dbias_y */

    kf->F_data[22] =  q2 * QEKF_INS.dt / 2;    /* dq3/dbias_x */
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;    /* dq3/dbias_y */

    /**
     * 渐消因子（Fading Filter）
     * P_data[28] 和 P_data[35] 是零偏估计的方差
     * 除以 lambda（<1）使方差增大，防止零偏协方差过度收敛
     * 这样即使零偏已经收敛，也能保持一定的调整能力
     */
    kf->P_data[28] /= QEKF_INS.lambda;  /* gyro_bias_x 方差 */
    kf->P_data[35] /= QEKF_INS.lambda;  /* gyro_bias_y 方差 */

    /* 零偏方差限幅：防止协方差矩阵发散 */
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;  /* 上限：10000 */
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;  /* 上限：10000 */
    }
}

/**
 * @brief  在工作点处计算观测函数 h(x) 的雅可比矩阵 H
 *         观测函数：h(x) = C_n2b * [0, 0, 1]'（从导航系到体坐标系的重力投影）
 * @param[in,out] kf  卡尔曼滤波器实例指针
 *
 * @note   H 矩阵为 3x6，其中后两列（零偏部分）为零
 *         H 矩阵描述了观测值（加速度）对各状态的偏导数
 *
 * H 矩阵结构（3x6）：
 *  0     1     2     3     4     5
 *  6     7     8     9    10    11
 * 12    13    14    15    16    17
 * 最后两列（index 4,5 和 10,11 和 16,17）为零
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;

    /* 预计算 2*qi */
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    /* 清零 H 矩阵 */
    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    /**
     * 设置 H 矩阵（观测函数的雅可比矩阵）
     * 观测值：重力在体坐标系下的投影 [ax, ay, az]
     * h(x) = [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0^2 - q1^2 - q2^2 + q3^2]
     */
    /* 第一行：dax/dq */
    kf->H_data[0] = -doubleq2;
    kf->H_data[1] =  doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] =  doubleq1;

    /* 第二行：day/dq */
    kf->H_data[6] =  doubleq1;
    kf->H_data[7] =  doubleq0;
    kf->H_data[8] =  doubleq3;
    kf->H_data[9] =  doubleq2;

    /* 第三行：daz/dq */
    kf->H_data[12] =  doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] =  doubleq3;
}

/**
 * @brief  利用观测值和先验估计得到最优后验估计
 *         包含卡方检验、自适应增益和发散保护
 * @param[in,out] kf  卡尔曼滤波器实例指针
 *
 * @note   此函数作为 User_Func3_f 替换标准卡尔曼滤波的 xhatUpdate 步骤
 *         算法流程：
 *           1. 计算 S = H*P'*H' + R
 *           2. 计算预测重力方向 h(xhat')
 *           3. 计算残差 z - h(xhat')
 *           4. 卡方检验判断量测是否可靠
 *           5. 根据检验结果决定：使用量测更新 / 仅预测 / 自适应增益
 *           6. 计算卡尔曼增益 K = P'*H'*S^-1
 *           7. 后验估计 xhat = xhat' + K*(z - h(xhat'))
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    /* 计算 H 矩阵的转置 HT（3x6 -> 6x3） */
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);

    /* temp_matrix = H * P' (3x6 * 6x6 = 3x6) */
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);

    /* temp_matrix1 = H * P' * HT (3x6 * 6x3 = 3x3) */
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);

    /* S = H * P' * HT + R (3x3) */
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);

    /* temp_matrix1 = inv(S) = inv(H * P' * HT + R) (3x3) */
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

    /* 保存预测四元数 */
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    /**
     * 计算预测的重力方向 h(xhat')
     * 从四元数推算体坐标系下的重力投影
     */
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);           /* 预测 ax */
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);           /* 预测 ay */
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;  /* 预测 az */

    /**
     * 计算方向余弦（预测重力方向与各轴的夹角）
     * 用于后续自适应增益调整：当重力方向与某轴平行时，该轴的修正效果更好
     */
    for (uint8_t i = 0; i < 3; ++i)
    {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    /* 计算残差：z - h(xhat') = 实际加速度 - 预测重力方向 */
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

    /* ========== 卡方检验（Chi-square test）========== */
    /* Chi-square = (z - h(xhat'))' * inv(S) * (z - h(xhat')) */
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);

    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector);

    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);

    /* 判断 1：卡方值很小，滤波器已收敛 */
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;  /* 标记为已收敛 */
    }

    /* 判断 2：卡方值超过阈值，但之前已收敛 */
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++;  /* 载体静止时仍无法通过检验，累计错误 */
        }
        else
        {
            QEKF_INS.ErrorCount = 0;  /* 载体运动时不过检验是正常的 */
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            /* 滤波器发散：重置收敛标志，允许 P 矩阵更新以重新收敛 */
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE;  /* 不跳过 P 矩阵更新 */
        }
        else
        {
            /**
             * 残差未通过卡方检验，且载体存在运动加速度
             * 此时量测值不可信，仅使用预测值（不进行量测更新）
             * xhat(k) = xhat'(k)
             * P(k) = P'(k)
             */
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE;  /* 跳过 P 矩阵更新 */
            return;
        }
    }
    else
    {
        /**
         * 判断 3：滤波器处于发散状态或卡方值可接受
         * 使用自适应增益：卡方值越小，增益越大（越信任量测）
         */
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
        {
            /* 自适应增益缩放因子：卡方值越大，缩放因子越小 */
            QEKF_INS.AdaptiveGainScale =
                (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0])
                / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1;  /* 完全信任量测 */
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;  /* 允许 P 矩阵更新 */
    }

    /* ========== 计算卡尔曼增益 K ========== */
    /* K = P' * HT * inv(S) */
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);  /* P' * HT */
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);  /* * inv(S) */

    /* ========== 自适应增益调整 ========== */
    /* 对所有增益元素乘以自适应缩放因子 */
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; ++i)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }

    /**
     * 对零偏估计部分的增益进行方向余弦加权
     * 当重力方向与某轴平行时，该轴的修正效果更好，增益更大
     * OrientationCosine[i] / (pi/2) 范围为 0~1
     */
    for (uint8_t i = 4; i < 6; ++i)
    {
        for (uint8_t j = 0; j < 3; ++j)
        {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f;
        }
    }

    /* 计算状态修正量：K * (z - h(xhat')) */
    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

    /* ========== 零漂修正限幅 ========== */
    /* 限制零偏修正量，防止异常值导致零偏发散 */
    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; ++i)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;  /* 上限 */
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt; /* 下限 */
            }
        }
    }

    /* 最终后验估计：xhat = xhat' + K * (z - h(xhat')) */
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief  EKF 观测环节回调函数
 *         在卡尔曼滤波更新过程中保存中间矩阵数据
 * @param[in] kf  卡尔曼滤波器实例指针
 *
 * @note   此函数作为 User_Func0_f 在量测更新前被调用
 *         将 P、K、H 矩阵数据保存到静态数组中
 *         可用于调试和监控滤波器状态
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));  /* 保存 P 矩阵 */
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));  /* 保存 K 矩阵 */
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));  /* 保存 H 矩阵 */
}

/**
 * @brief  陀螺仪坐标变换初始化
 *         计算从陀螺仪坐标系到云台坐标系的 3x3 旋转矩阵
 * @param[in,out] gim_trans 云台变换参数结构体指针
 *
 * @note   与 bmi.c 中的 transform_init() 功能相同
 *         若不需要坐标变换，可在 imu_sensor.c 中 imu_init 将调用注释掉
 */
void transform_init(gimbal_transform_t *gim_trans)
{
    float arz, ary, arx;

    /* 角度单位转换：度 -> 弧度 */
    arz = gim_trans->arz * (double)0.017453;
    ary = gim_trans->ary * (double)0.017453;
    arx = gim_trans->arx * (double)0.017453;

    /**
     * 旋转矩阵赋值（Z-Y-X 欧拉角旋转矩阵）
     * R = Rz(arz) * Ry(ary) * Rx(arx)
     */
    gim_trans->trans[0] = arm_cos_f32(arz)*arm_cos_f32(ary);
    gim_trans->trans[1] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) - arm_sin_f32(arz)*arm_cos_f32(arx);
    gim_trans->trans[2] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) + arm_sin_f32(arz)*arm_sin_f32(arx);
    gim_trans->trans[3] = arm_sin_f32(arz)*arm_cos_f32(ary);
    gim_trans->trans[4] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) + arm_cos_f32(arz)*arm_cos_f32(arx);
    gim_trans->trans[5] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) - arm_cos_f32(arz)*arm_sin_f32(arx);
    gim_trans->trans[6] = -arm_sin_f32(ary);
    gim_trans->trans[7] = arm_cos_f32(ary)*arm_sin_f32(arx);
    gim_trans->trans[8] = arm_cos_f32(ary)*arm_cos_f32(arx);

    /* 初始化 3x3 变换矩阵实例 */
    arm_mat_init_f32(&EKFTrans, 3, 3, (float *)gim_trans->trans);
}

/**
 * @brief  将陀螺仪坐标变换为云台坐标
 *         对角速度和加速度分别进行坐标变换
 * @param[in]  gx, gy, gz     陀螺仪角速度原始数据
 * @param[in]  ax, ay, az     加速度计原始数据
 * @param[out] ggx, ggy, ggz 变换后的角速度（云台坐标系）
 * @param[out] aax, aay, aaz 变换后的加速度（云台坐标系）
 *
 * @note   与 bmi.c 中的 Vector_Transform() 功能相同
 */
void Vector_Transform(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float *ggx, float *ggy, float *ggz,
                      float *aax, float *aay, float *aaz)
{
    /* 陀螺仪数据输入输出数组 */
    float gyro_in[3], gyro_out[3];
    /* 加速度计数据输入输出数组 */
    float acc_in[3], acc_out[3];

    /* 填充陀螺仪输入向量 */
    gyro_in[0] = (float)gx;
    gyro_in[1] = (float)gy;
    gyro_in[2] = (float)gz;

    /* 填充加速度计输入向量 */
    acc_in[0] = (float)ax;
    acc_in[1] = (float)ay;
    acc_in[2] = (float)az;

    /* 陀螺仪坐标变换：gyro_out = gyro_in * EKFTrans */
    arm_mat_init_f32(&EKFSrc, 1, 3, gyro_in);
    arm_mat_init_f32(&EKFDst, 1, 3, gyro_out);
    arm_mat_mult_f32(&EKFSrc, &EKFTrans, &EKFDst);
    *ggx = gyro_out[0];
    *ggy = gyro_out[1];
    *ggz = gyro_out[2];

    /* 加速度计坐标变换：acc_out = acc_in * EKFTrans */
    arm_mat_init_f32(&EKFSrc, 1, 3, acc_in);
    arm_mat_init_f32(&EKFDst, 1, 3, acc_out);
    arm_mat_mult_f32(&EKFSrc, &EKFTrans, &EKFDst);
    *aax = acc_out[0];
    *aay = acc_out[1];
    *aaz = acc_out[2];
}

/**
 * @brief  获取世界坐标系（地面坐标系）的加速度
 *         将 IMU 体坐标系的加速度转换为世界坐标系
 * @param[in]  pitch, roll, yaw  当前欧拉角（度）
 * @param[in]  ax, ay, az        体坐标系下的加速度
 * @param[out] accx, accy, accz  世界坐标系下的加速度
 *
 * @note   与 bmi.c 中的 BMI_Get_Acceleration() 功能相同
 */
void BMI_Get_Acceleration(float pitch, float roll, float yaw,
                          float ax, float ay, float az,
                          float *accx, float *accy, float *accz)
{
    float imu_accx, imu_accy, imu_accz;

    /* 角度制转弧度制 */
    pitch *= (double)0.017453;
    yaw   *= (double)0.017453;
    roll  *= (double)0.017453;

    /* 补偿重力分量 */
    imu_accx = ax + arm_sin_f32(pitch) * GRAVITY_EARTH;
    imu_accy = ay - arm_sin_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;
    imu_accz = az - arm_cos_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;

    /* 通过欧拉角旋转矩阵转换到世界坐标系 */
    *accx = imu_accx * arm_cos_f32(pitch) + imu_accz * arm_sin_f32(pitch);
    *accy = imu_accy * arm_cos_f32(roll) - imu_accz * arm_sin_f32(roll);
    *accz = imu_accz * arm_cos_f32(pitch) * arm_cos_f32(roll)
          - imu_accx * arm_sin_f32(pitch) * arm_cos_f32(roll)
          + imu_accy * arm_sin_f32(roll) * arm_cos_f32(pitch);
}

#endif  /* IMU_USE_EKF */
