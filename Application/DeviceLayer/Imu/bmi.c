/**
 * @file    bmi.c
 * @brief   陀螺仪数据解算 - 基于 Mahony 互补滤波算法
 *
 * 本文件实现了基于 Mahony 互补滤波器的 IMU 姿态解算功能。
 * Mahony 算法通过融合陀螺仪和加速度计数据，利用四元数表示姿态，
 * 并通过比例-积分（PI）控制器修正陀螺仪漂移。
 *
 * 主要功能：
 *   1. 坐标系变换（陀螺仪坐标系 -> 云台坐标系）
 *   2. 四元数姿态更新（Mahony 互补滤波）
 *   3. 欧拉角提取（pitch, roll, yaw）
 *   4. 世界坐标系加速度计算
 *   5. Kp 参数自适应调整
 *
 * @note   需要在 rp_config.h 中设置 IMU_USE_MAHONY==1 才会编译本文件
 * @update 2024-8
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bmi.h"        /* 姿态解算头文件，包含 bmi_t 结构体和函数声明 */
#include "rp_math.h"    /* 数学库，包含三角函数、矩阵运算等 */
#include "ave_filter.h" /* 均值滤波器，用于数据平滑 */

#if IMU_USE_MAHONY==1  /* 条件编译：仅在使用 Mahony 算法时编译 */

/* Exported macro ------------------------------------------------------------*/
/* Macros to select the sensors */

/** @brief 标准重力加速度 (m/s^2) */
#define GRAVITY_EARTH  (9.80665f)

/* Private variables ---------------------------------------------------------*/
/**
 * @brief ARM CMSIS-DSP 矩阵实例
 * Trans: 3x3 坐标变换矩阵
 * Src:   源向量矩阵（1x3）
 * Dst:   目标向量矩阵（1x3）
 */
arm_matrix_instance_f32 Trans;  /* 变换矩阵 */
arm_matrix_instance_f32 Src;    /* 源向量 */
arm_matrix_instance_f32 Dst;    /* 目标向量 */

/**
 * @brief 云台坐标变换参数结构体
 *
 * 坐标变换采用 Z-Y-X 欧拉角描述，即从陀螺仪坐标系向云台坐标系变换中，
 * 坐标系按照绕陀螺仪 Z 轴、Y 轴、X 轴的顺序旋转
 * 每一次旋转的参考坐标系为当前陀螺仪坐标系（本体旋转）
 *
 * @arz 陀螺仪 X 轴与 roll 轴之间的夹角（度），默认 90 度
 * @ary 陀螺仪 X 轴与 yaw 轴之间的夹角（度），默认 0 度
 * @arx 陀螺仪 Y 轴与 yaw 轴之间的夹角（度），默认 0 度
 */
gimbal_transform_t gim_trans = {
    .arz = 180.0f,   /* Z 轴旋转角度 */
    .ary = 180.0f,    /* Y 轴旋转角度 */
    .arx = 0.0f,    /* X 轴旋转角度：0度 */
    .trans = {0.0f}, /* 变换矩阵数组（3x3 = 9个元素） */
};

/**
 * @brief Mahony 姿态解算核心参数结构体
 *
 * @Kp   比例增益：越大表示越信任加速度计数据
 *        Kp 越大，初始化时数据稳定越快，但快速晃动时 yaw 轴可能漂移
 * @halfT 解算周期的一半（秒）
 *        例如：1ms 解算一次则 halfT = 0.0005f
 * @norm  临时归一化变量
 * @gx,gy,gz  陀螺仪角速度数据（rad/s）
 * @ax,ay,az  加速度计数据（m/s^2）
 * @vx,vy,vz  四元数解算出的参考重力方向
 * @ex,ey,ez  加速度计与参考重力方向的误差（叉积）
 * @q0-q3     四元数（姿态表示）
 */
bmi_t bmi = {
    .Kp = 0.125f,           /* 比例增益，初始值 0.125 */
    .norm = 0.0f,           /* 归一化中间变量 */
    .halfT = 0.00025f,      /* 解算周期一半：0.25ms（对应 2kHz 采样率） */
    .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,  /* 陀螺仪原始数据 */
    .ax = 0.0f, .ay = 0.0f, .az = 0.0f,  /* 加速度计原始数据 */
    .vx = 0.0f, .vy = 0.0f, .vz = 0.0f,  /* 四元数推算的重力方向 */
    .ex = 0.0f, .ey = 0.0f, .ez = 0.0f,  /* 误差补偿量 */
    /* 四元数初始值：q0=1 表示无旋转 */
    .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f,
    .q0_temp = 0.0f, .q1_temp = 0.0f, .q2_temp = 0.0f, .q3_temp = 0.0f,
    .sintemp = 0.0f, .sintemp_ = 0.0f, .costemp = 0.0f, .costemp_ = 0.0f,
};

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  快速开平方倒数算法（Quake III 经典算法）
 * @param  x 输入值
 * @return 1/sqrt(x) 的近似值
 *
 * @note   使用位操作实现快速计算，精度约 1%
 *         算法原理：利用 IEEE 754 浮点数的指数部分进行估算
 *         i = 0x5f3759df - (i >> 1) 是算法的核心魔法数
 */
float inVSqrt(float x)
{
	float halfx = 0.5f * x;  /* 预计算 x/2 */
	float y = x;              /* y 既作为输入也作为输出 */
	long i = *(long*)&y;      /* 将浮点数位模式解释为整数 */
	/* 核心算法：利用浮点数指数部分的近似线性关系 */
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;          /* 将整数位模式解释回浮点数 */
	/* 牛顿迭代法精修一次，提高精度 */
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
 * @brief  陀螺仪坐标变换初始化
 *         计算从陀螺仪坐标系到云台坐标系的 3x3 旋转矩阵
 * @param  gim_trans 云台变换参数结构体指针（输入角度，输出变换矩阵）
 *
 * @note   若不需要坐标变换，可在 imu_sensor.c 中 imu_init 将调用注释掉
 *         旋转矩阵采用 Z-Y-X 欧拉角顺序（本体旋转）
 */
void transform_init(gimbal_transform_t *gim_trans)
{
    float arz, ary, arx;

	/* 角度单位转换：度 -> 弧度（乘以 pi/180 ≈ 0.017453） */
	arz = gim_trans->arz * (double)0.017453;
	ary = gim_trans->ary * (double)0.017453;
	arx = gim_trans->arx * (double)0.017453;

	/**
	 * 旋转矩阵赋值（Z-Y-X 欧拉角旋转矩阵）
	 * R = Rz(arz) * Ry(ary) * Rx(arx)
	 * 矩阵元素排列为行优先（row-major）：
	 * [0] [1] [2]
	 * [3] [4] [5]
	 * [6] [7] [8]
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

    /* 初始化 3x3 变换矩阵实例（ARM CMSIS-DSP 格式） */
	arm_mat_init_f32(&Trans, 3, 3, (float *)gim_trans->trans);
}

/**
 * @brief  将陀螺仪坐标变换为云台坐标
 *         对角速度和加速度分别进行坐标变换
 * @param[in]  gx, gy, gz     陀螺仪角速度原始数据
 * @param[in]  ax, ay, az     加速度计原始数据
 * @param[out] ggx, ggy, ggz 变换后的角速度（云台坐标系）
 * @param[out] aax, aay, aaz 变换后的加速度（云台坐标系）
 *
 * @note   坐标变换采用 Z-Y-X 欧拉角描述
 *         若不需要变换可在 imu_protocol.c 中 imu_update 将调用注释掉
 */
void Vector_Transform(float gx, float gy, float gz,\
                      float ax, float ay, float az,\
                      float *ggx, float *ggy, float *ggz,\
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

	/* 陀螺仪坐标变换：gyro_out = gyro_in * Trans（1x3 乘以 3x3） */
	arm_mat_init_f32(&Src, 1, 3, gyro_in);
	arm_mat_init_f32(&Dst, 1, 3, gyro_out);
	arm_mat_mult_f32(&Src, &Trans, &Dst);  /* 矩阵乘法 */
	*ggx = gyro_out[0];
	*ggy = gyro_out[1];
	*ggz = gyro_out[2];

	/* 加速度计坐标变换：acc_out = acc_in * Trans（1x3 乘以 3x3） */
	arm_mat_init_f32(&Src, 1, 3, acc_in);
	arm_mat_init_f32(&Dst, 1, 3, acc_out);
	arm_mat_mult_f32(&Src, &Trans, &Dst);  /* 矩阵乘法 */
	*aax = acc_out[0];
	*aay = acc_out[1];
	*aaz = acc_out[2];
}

/**
 * @brief  获取欧拉角（Mahony 互补滤波算法核心函数）
 *         融合陀螺仪和加速度计数据，更新四元数并提取欧拉角
 * @param[out] pitch, roll, yaw  输出的欧拉角（度）
 * @param[in,out] gx, gy, gz     陀螺仪角速度（输入，会被误差补偿修改）
 * @param[in,out] ax, ay, az     加速度计数据（输入）
 * @return 0 表示成功
 *
 * @note   不带 _ 的输出涉及加速度计修正
 *         带 _ 的为纯陀螺仪积分结果，用于差分计算速度（更平滑）
 *         算法流程：
 *           1. 归一化加速度计数据
 *           2. 用四元数推算当前重力方向
 *           3. 计算加速度计与推算重力的误差（叉积）
 *           4. 用 PI 控制器修正陀螺仪数据
 *           5. 四元数一阶积分更新
 *           6. 四元数归一化
 *           7. 提取欧拉角
 */
uint8_t BMI_Get_EulerAngle(float *pitch,float *roll,float *yaw,\
                           float *gx,float *gy,float *gz,\
                           float *ax,float *ay,float *az)
{
	/* 将角速度数据赋值到 bmi 结构体 */
    bmi.gx = *gx;
    bmi.gy = *gy;
    bmi.gz = *gz;

	/* 将加速度数据赋值到 bmi 结构体 */
    bmi.ax = *ax;
    bmi.ay = *ay;
    bmi.az = *az;

	/* ========== Mahony 互补滤波算法 start ========== */

	/* 检查加速度计数据是否有效（非零） */
	if(bmi.ax * bmi.ay * bmi.az != 0)
	{
        /* 步骤1：加速度计数据归一化（使其模长为 1） */
        bmi.norm = inVSqrt(bmi.ax*bmi.ax + bmi.ay*bmi.ay + bmi.az*bmi.az);
        bmi.ax *= bmi.norm;  /* ax = ax / |a| */
        bmi.ay *= bmi.norm;  /* ay = ay / |a| */
        bmi.az *= bmi.norm;  /* az = az / |a| */

        /**
         * 步骤2：用当前四元数推算重力方向（载体坐标系下的参考重力）
         * 这是从体坐标系到参考坐标系的旋转矩阵第三列
         * vx = -sin(Pitch)
         * vy = sin(Roll)*cos(Pitch)
         * vz = cos(Roll)*cos(Pitch)
         */
        bmi.vx = -2*(bmi.q1*bmi.q3 - bmi.q0*bmi.q2);
        bmi.vy = -2*(bmi.q0*bmi.q1 + bmi.q2*bmi.q3);
        bmi.vz = -(bmi.q0*bmi.q0 - bmi.q1*bmi.q1 - bmi.q2*bmi.q2 + bmi.q3*bmi.q3);

        /**
         * 步骤3：计算加速度计与推算重力的误差（叉积）
         * 当角度较小时，叉积近似等于两向量夹角的正弦值
         * e = a × v（加速度计方向 × 推算重力方向）
         */
        bmi.ex = (bmi.az*bmi.vy - bmi.ay*bmi.vz);
        bmi.ey = (bmi.ax*bmi.vz - bmi.az*bmi.vx);
        bmi.ez = (bmi.ay*bmi.vx - bmi.ax*bmi.vy);

        /**
         * 步骤4：PI 控制器修正陀螺仪数据
         * Kp 越大，越信任加速度计，修正力度越大
         * 修正后的角速度 = 原始角速度 + Kp * 误差
         */
        bmi.gx = bmi.gx + bmi.ex*bmi.Kp;
        bmi.gy = bmi.gy + bmi.ey*bmi.Kp;
        bmi.gz = bmi.gz + bmi.ez*bmi.Kp;
	}

    /* 步骤5：保存当前四元数到临时变量（防止更新过程中数据被覆盖） */
    bmi.q0_temp = bmi.q0;
    bmi.q1_temp = bmi.q1;
    bmi.q2_temp = bmi.q2;
    bmi.q3_temp = bmi.q3;

    /**
     * 步骤6：四元数一阶积分更新（龙格-库塔法的简化版本）
     * q(n+1) = q(n) + 0.5 * q(n) * omega * dt
     * 其中 omega = [0, gx, gy, gz]，dt = 2*halfT
     */
    bmi.q0 = bmi.q0_temp + (-bmi.q1_temp*bmi.gx - bmi.q2_temp*bmi.gy -bmi.q3_temp*bmi.gz)*bmi.halfT;
    bmi.q1 = bmi.q1_temp + ( bmi.q0_temp*bmi.gx + bmi.q2_temp*bmi.gz -bmi.q3_temp*bmi.gy)*bmi.halfT;
    bmi.q2 = bmi.q2_temp + ( bmi.q0_temp*bmi.gy - bmi.q1_temp*bmi.gz +bmi.q3_temp*bmi.gx)*bmi.halfT;
    bmi.q3 = bmi.q3_temp + ( bmi.q0_temp*bmi.gz + bmi.q1_temp*bmi.gy -bmi.q2_temp*bmi.gx)*bmi.halfT;

    /**
     * 步骤7：四元数归一化
     * 防止数值积分累积误差导致四元数模长偏离 1
     */
    bmi.norm = inVSqrt(bmi.q0*bmi.q0 + bmi.q1*bmi.q1 + bmi.q2*bmi.q2 + bmi.q3*bmi.q3);
    bmi.q0 *= bmi.norm;
    bmi.q1 *= bmi.norm;
    bmi.q2 *= bmi.norm;
    bmi.q3 *= bmi.norm;

    /**
     * 步骤8：从四元数提取欧拉角
     * 使用 atan2 而非 asin 可避免万向节锁问题
     * asin(x) = atan(x / sqrt(1-x*x))
     */
    /* 计算 pitch 的 sin 值和 cos 值 */
    bmi.sintemp = 2*bmi.q1*bmi.q3 - 2*bmi.q0*bmi.q2;
    arm_sqrt_f32(1 - bmi.sintemp*bmi.sintemp, &bmi.costemp);

    /* 提取欧拉角（弧度） */
    /* pitch = -asin(2*q1*q3 - 2*q0*q2) */
    arm_atan2_f32(bmi.sintemp, bmi.costemp, pitch);
    /* yaw = atan2(2*(q1*q2 + q0*q3), q0^2 + q1^2 - q2^2 - q3^2) */
    arm_atan2_f32(2*(bmi.q1*bmi.q2 + bmi.q0*bmi.q3),
                     bmi.q0*bmi.q0 + bmi.q1*bmi.q1 - bmi.q2*bmi.q2 - bmi.q3*bmi.q3, yaw);
    /* roll = atan2(2*q2*q3 + 2*q0*q1, q0^2 - q1^2 - q2^2 + q3^2) */
    arm_atan2_f32(2*bmi.q2*bmi.q3 + 2*bmi.q0*bmi.q1,
                    bmi.q0*bmi.q0 - bmi.q1*bmi.q1 - bmi.q2*bmi.q2 + bmi.q3*bmi.q3, roll);

    /* 弧度转角度制（乘以 180/pi ≈ 57.295773） */
	*roll  *=  57.295773f;
	*pitch *= -57.295773f;  /* pitch 取负号，符合右手坐标系习惯 */
	*yaw   *=  57.295773f;
	/* ========== Mahony 互补滤波算法 end ========== */

	return 0;
}

/**
 * @brief  获取世界坐标系（地面坐标系）的加速度
 *         将 IMU 体坐标系的加速度转换为世界坐标系
 * @param[in]  pitch, roll, yaw  当前欧拉角（度）
 * @param[in]  ax, ay, az        体坐标系下的加速度（已去除重力分量）
 * @param[out] accx, accy, accz  世界坐标系下的加速度
 *
 * @note   转换过程：
 *           1. 先将加速度计数据中的重力分量补偿掉
 *           2. 再通过欧拉角旋转矩阵转换到世界坐标系
 */
void BMI_Get_Acceleration(float pitch, float roll, float yaw,\
                          float ax, float ay, float az,\
                          float *accx, float *accy, float *accz)
{
	float imu_accx, imu_accy, imu_accz;

    /* 角度制转弧度制 */
	pitch *= (double)0.017453;
	yaw   *= (double)0.017453;
	roll  *= (double)0.017453;

	/* 补偿重力分量（从加速度计原始数据中减去重力在各轴的投影） */
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

/**
 * @brief  Mahony 滤波器 Kp 参数自适应调整
 *         根据系统运行时间动态调整 Kp 值
 * @note   上电后 1 秒内 Kp = 1.0（快速收敛）
 *         1 秒后 Kp = 0.1（稳定运行，减少加速度计影响）
 */
void BMI_Change_Kp(void)
{
    float *kp = &bmi.Kp;

	if(HAL_GetTick() <= 1000)
	{
		*kp = 1.0f;  /* 启动阶段：高增益，快速收敛 */
	}
	else if(HAL_GetTick() > 1000)
	{
		*kp = 0.1f;  /* 稳定阶段：低增益，减少加速度计噪声影响 */
	}
}

#endif  /* IMU_USE_MAHONY */
