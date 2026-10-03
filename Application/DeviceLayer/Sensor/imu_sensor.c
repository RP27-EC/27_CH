/**
  ******************************************************************************
  * @file           : imu_sensor.c
  * @brief          : IMU惯性测量单元传感器管理层
  * @details        : 管理BMI088 IMU传感器的完整工作流程，包括：
  *                   - 传感器初始化（BMI088驱动 + 姿态解算算法初始化）
  *                   - 数据读取与坐标系变换
  *                   - 陀螺仪零偏校准（上电自动校准）
  *                   - 原始数据低通滤波
  *                   - 姿态解算（支持Mahony互补滤波和扩展卡尔曼滤波EKF两种算法）
  *                   - 世界坐标系加速度计算
  *                   - 角速度数据平滑滤波
  *                   - 设备心跳检测和数据有效性判断
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"
#include "drv_gpio.h"
#include "drv_tick.h"

/* 根据编译宏选择姿态解算算法 */
#if IMU_USE_EKF == 1
#include "bmi_EKF.h"      /* 扩展卡尔曼滤波算法头文件 */
#endif
#if IMU_USE_MAHONY == 1
#include "bmi.h"           /* Mahony互补滤波算法头文件 */
#endif

/* Private function prototypes -----------------------------------------------*/
void imu_init(imu_sensor_t *self);                    /* IMU初始化 */
void imu_heart_beat(work_state_t *heart);              /* 心跳检测 */
void imu_update(imu_sensor_t *self);                   /* 数据更新与解算 */
void imu_set_temperature(imu_sensor_t *self, float temp); /* 温度设置 */
#if IMU_USE_EKF == 1
static void InitQuaternion(float *init_q4);            /* 四元数初始化（EKF专用） */
#endif

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  IMU温度控制PID控制器
  * @details 用于控制IMU芯片温度恒定（部分应用需要恒温以提高精度）
  *          - kp = 1000: 比例系数
  *          - ki = 0: 积分系数（未使用）
  *          - out_max = 4500: 输出限幅（对应PWM值）
  */
pid_ctrl_t imu_temp_pid = {
    .kp = 1000.0f,
    .ki = 0.0f,
    .out_max = 4500,
};

/**
  * @brief  IMU信息全局变量
  * @details 存储IMU的零偏校准值和初始化标志
  *          - gx_offset/gy_offset/gz_offset: 陀螺仪三轴零偏值（上电校准得到）
  *          - init_flag: 初始化完成标志（1=已初始化）
  */
imu_info_t imu_info =
{
	.offset_info.gx_offset = 0.f,  /* X轴陀螺仪零偏 */
	.offset_info.gy_offset = 0.f,  /* Y轴陀螺仪零偏 */
	.offset_info.gz_offset = 0.f,  /* Z轴陀螺仪零偏 */
	.init_flag = 0,                /* 初始化标志，0=未初始化 */
};

/* Exported variables --------------------------------------------------------*/

/**
  * @brief  IMU传感器对象（全局唯一实例）
  * @details 绑定所有操作函数和配置参数：
  *          - info: 指向IMU信息（零偏、角度等）
  *          - driver: 驱动类型为SPI2
  *          - work_state: 工作状态（离线/在线/错误码等）
  *          - id: 设备ID
  *          - temp_pid: 温度控制PID
  *          - init/update/heart_beat/set_temperature: 操作函数指针
  */
imu_sensor_t imu_sensor = {
	.info = &imu_info,                /* 绑定IMU信息 */
	.driver.tpye = DR_SPI2,           /* 使用SPI2接口 */
	.work_state.dev_state = DEV_OFFLINE,  /* 初始状态为离线 */
	.id = DEV_ID_IMU,                 /* 设备ID */
	.work_state.cali_end = 0,         /* 校准未完成 */
	.work_state.offline_max_cnt = 50, /* 离线最大计数50次 */
	.work_state.err_cnt = 0,          /* 错误计数初始为0 */
    .temp_pid = &imu_temp_pid,        /* 绑定温度PID控制器 */
	.init = &imu_init,                /* 绑定初始化函数 */
	.update = &imu_update,            /* 绑定数据更新函数 */
    .heart_beat = &imu_heart_beat,    /* 绑定心跳检测函数 */
    .set_temperature = &imu_set_temperature, /* 绑定温度设置函数 */
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* 临时读取缓冲区（调试用） */
float imu_read[3];

/* 初始化失败重试次数上限 */
uint8_t init_cnt = 200;

/**
  * @brief  IMU传感器初始化函数
  * @param  self: IMU传感器对象指针
  * @retval None
  * @details 初始化流程：
  *          1. 调用BMI088_init()初始化底层硬件
  *          2. 若初始化失败，自动重试最多200次
  *          3. 重试次数耗尽后触发系统复位（NVIC_SystemReset）
  *          4. 初始化成功后：
  *             a. 标记设备在线
  *             b. 根据编译宏选择初始化Mahony或EKF姿态解算算法
  *             c. 初始化坐标变换矩阵
  *             d. 陀螺仪零偏校准状态设为"开始校准"
  */
void imu_init(struct imu_struct *self)
{
	uint32_t tickstart = HAL_GetTick();

	/* 初始状态设为离线 */
	self->work_state.dev_state = DEV_OFFLINE;

	/* 调用BMI088底层驱动初始化（含SPI通信和寄存器配置） */
	self->work_state.init_code = BMI088_init();

	/* 初始化失败时自动重试 */
	while(self->work_state.init_code)
	{
		/* 重试次数超过上限，触发系统复位 */
		if (++self->work_state.err_cnt == init_cnt)
		{
			__set_FAULTMASK(1);    /* 禁用所有中断 */
			NVIC_SystemReset();     /* 系统复位重启 */
			break;
		}
        self->work_state.err_code = IMU_INIT_ERR;  /* 设置错误码 */
        self->work_state.init_code = BMI088_init(); /* 重新尝试初始化 */
	}

	/* 初始化成功 */
	if (self->work_state.init_code == 0)
	{
		self->work_state.dev_state = DEV_ONLINE;  /* 标记设备在线 */
		self->info->init_flag = 1;                 /* 设置初始化完成标志 */

		/* 根据编译宏选择姿态解算算法 */
#if IMU_USE_MAHONY == 1
		/* Mahony互补滤波算法：初始化坐标变换矩阵 */
		transform_init(&gim_trans);
#endif

#if IMU_USE_EKF == 1
		/* 扩展卡尔曼滤波算法：初始化坐标变换矩阵和四元数 */
		transform_init(&EKFgim_trans);

		/* 使用加速度计数据初始化四元数（Roll/Pitch由重力方向确定，Yaw=0） */
		float init_quaternion[4] = {0};
		InitQuaternion(init_quaternion);

		/* EKF初始化：过程噪声Q1=10, Q2=0.001, 观测噪声R=8000000, 渐消因子lambda=1 */
		IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 8000000, 1);
#endif

		/* 设置错误码为"数据校准中"，触发后续的零偏校准流程 */
		self->work_state.err_code = IMU_DATA_CALI;

		/* 清零陀螺仪零偏值 */
		imu_sensor.info->offset_info.gx_offset = 0.f;
		imu_sensor.info->offset_info.gy_offset = 0.f;
		imu_sensor.info->offset_info.gz_offset = 0.f;
	}
	else
	{
		/* 初始化彻底失败 */
		self->work_state.dev_state = DEV_OFFLINE;
		self->work_state.err_code = IMU_INIT_ERR;
		self->work_state.offline_cnt = self->work_state.offline_max_cnt;
		self->info->init_flag = 0;
	}
}

/**
  * @brief  IMU心跳检测函数
  * @param  heart: 工作状态结构体指针
  * @retval None
  * @details 周期性调用，累加离线计数器。当收到有效数据时（在update中），
  *          offline_cnt会被清零。超过阈值则判定设备离线。
  */
void imu_heart_beat(work_state_t *heart)
{
	/* 离线计数器递增 */
	heart->offline_cnt++;

	/* 超过最大离线计数阈值 */
	if(heart->offline_cnt > heart->offline_max_cnt)
	{
		heart->offline_cnt = heart->offline_max_cnt;  /* 钳位防止溢出 */
		heart->dev_state = DEV_OFFLINE;                /* 标记设备离线 */
	}
	else
	{
		/* 未超阈值，若当前为离线状态则恢复在线 */
		if(heart->dev_state == DEV_OFFLINE)
        {
            heart->dev_state = DEV_ONLINE;
        }
	}
}

/**
  * @brief  设置IMU芯片温度（当前已禁用）
  * @param  self: IMU传感器对象指针
  * @param  temp: 目标温度值
  * @note   该功能已注释掉，当前不使用温度控制
  *          原设计通过PID控制加热电阻维持IMU恒温以提高精度
  */
void imu_set_temperature(imu_sensor_t *self, float temp)
{
	/* 温度控制功能已禁用 */
}

/* 角速度滑动平均滤波器（用于平滑角速度输出） */
ave_filter_t imu_pitch_dif_speed_ave_filter;  /* pitch轴角速度平均滤波 */
ave_filter_t imu_roll_dif_speed_ave_filter;   /* roll轴角速度平均滤波 */
ave_filter_t imu_yaw_dif_speed_ave_filter;    /* yaw轴角速度平均滤波 */

/* 临时变量（用于坐标变换前后的中间数据存储） */
static float pitch, roll, yaw;           /* 姿态角（度） */
static float gyrox, gyroy, gyroz;        /* 坐标变换后的角速度（rad/s） */
static float accx, accy, accz;           /* 坐标变换后的加速度（m/s²） */
static float gyrox_, gyroy_, gyroz_;     /* 低通滤波后的角速度 */
static float accx_, accy_, accz_;        /* 低通滤波后的加速度 */
static float gyro[3], accel[3], temp;    /* 原始传感器数据 */
static int16_t imu_cnt = 0;             /* 零偏校准计数器 */

#if IMU_USE_EKF == 1
/* EKF算法使用的采样时间相关变量 */
static float imu_dt;                     /* 采样周期（秒） */
static uint32_t imu_tick_now, imu_tick_last;  /* 时间戳记录 */
#endif

/**
  * @brief  IMU数据更新与姿态解算（核心函数）
  * @param  imu_sen: IMU传感器对象指针
  * @retval None
  * @details 完整的IMU数据处理流水线：
  *
  *          第一阶段：数据采集
  *          1. 从BMI088读取原始三轴角速度、加速度和温度
  *
  *          第二阶段：坐标系变换
  *          2. 将传感器坐标系转换为云台/底盘坐标系
  *
  *          第三阶段：零偏校准
  *          3. 上电前2000个周期进行陀螺仪零偏自校准
  *          4. 校准完成后将零偏值补偿到角速度数据
  *
  *          第四阶段：数据滤波
  *          5. 对角速度和加速度进行低通滤波，抑制高频噪声
  *
  *          第五阶段：姿态解算
  *          6. 根据编译宏选择Mahony或EKF算法进行姿态解算
  *
  *          第六阶段：后处理
  *          7. 计算世界坐标系加速度
  *          8. 计算角速度并进行平均滤波
  *          9. 检测数据有效性，判断设备是否正常
  */
void imu_update(imu_sensor_t *imu_sen)
{
    imu_info_t *imu_info = imu_sen->info;

	/* ==================== 第一阶段：数据采集 ==================== */
	/* 从BMI088读取三轴角速度、加速度和温度 */
	BMI088_read(gyro, accel, &temp);

	/* 存储原始数据到信息结构体 */
	imu_info->raw_info.acc_x = accel[0];
	imu_info->raw_info.acc_y = accel[1];
	imu_info->raw_info.acc_z = accel[2];
	imu_info->raw_info.gyro_x = gyro[0];
	imu_info->raw_info.gyro_y = gyro[1];
	imu_info->raw_info.gyro_z = gyro[2];

	/* ==================== 第二阶段：坐标系变换 ==================== */
	/* 将传感器坐标系数据变换到云台/底盘坐标系 */
	Vector_Transform(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],
	                 &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);

	/* ==================== 第三阶段：零偏校准 ==================== */
	/* 当错误码为IMU_DATA_CALI时，处于校准模式 */
	if (imu_sen->work_state.err_code == IMU_DATA_CALI)
	{
		if (imu_cnt < 2000)
		{
			/* 使用滑动平均法累积零偏值（步长0.0005，约2000次收敛） */
			imu_info->offset_info.gx_offset -= gyrox * 0.0005f;
			imu_info->offset_info.gy_offset -= gyroy * 0.0005f;
			imu_info->offset_info.gz_offset -= gyroz * 0.0005f;
			imu_cnt++;
		}
		else
		{
			/* 校准完成（2000次采样后） */
			imu_cnt = 0;
			imu_sen->work_state.err_code = IMU_NONE_ERR;  /* 清除校准状态 */

			/* 零偏值有效性检查：如果零偏过大（>0.005），认为异常并清零 */
			if (abs(imu_info->offset_info.gx_offset) > 0.005f)
				imu_info->offset_info.gx_offset = 0;
			if (abs(imu_info->offset_info.gy_offset) > 0.005f)
				imu_info->offset_info.gy_offset = 0;
			if (abs(imu_info->offset_info.gz_offset) > 0.005f)
				imu_info->offset_info.gz_offset = 0;

			imu_sen->work_state.cali_end = 1;  /* 标记校准完成 */
		}
	}
	else
	{
		/* 校准完成后，将零偏值补偿到角速度数据 */
#if IMU_USE_EKF == 1
		/* EKF模式：只修正Z轴零偏（X/Y轴零偏在EKF内部估计） */
		gyroz += imu_info->offset_info.gz_offset;
#endif

#if IMU_USE_MAHONY == 1
		/* Mahony模式：修正所有三轴零偏 */
		gyrox += imu_info->offset_info.gx_offset;
		gyroy += imu_info->offset_info.gy_offset;
		gyroz += imu_info->offset_info.gz_offset;
#endif
	}

	/* ==================== 第四阶段：数据滤波 ==================== */
	/* 对角速度进行低通滤波（滤波系数1表示基本不滤波，保留快速响应） */
	gyrox_ = Lowpass(gyrox_, gyrox, 1);
	gyroy_ = Lowpass(gyroy_, gyroy, 1);
	gyroz_ = Lowpass(gyroz_, gyroz, 1);

	/* 对加速度进行低通滤波（系数较小，滤波更强，抑制震动噪声） */
	accx_ = Lowpass(accx_, accx, 0.8);
	accy_ = Lowpass(accy_, accy, 0.2);
	accz_ = Lowpass(accz_, accz, 0.2);

	/* ==================== 第五阶段：姿态解算 ==================== */

#if IMU_USE_MAHONY == 1
	/* Mahony互补滤波姿态解算：输入滤波后的角速度和加速度，输出欧拉角 */
	BMI_Get_EulerAngle(&imu_info->base_info.pitch, &imu_info->base_info.roll, &imu_info->base_info.yaw,
	                   &gyrox_, &gyroy_, &gyroz_,
	                   &accx_, &accy_, &accz_);
#endif

#if IMU_USE_EKF == 1
	/* 扩展卡尔曼滤波姿态解算 */

	/* 计算采样时间间隔（微秒转秒） */
	imu_tick_now = micros();
	if(imu_tick_last == 0)  /* 第一次调用的特殊处理 */
	{
		imu_tick_last = imu_tick_now - 1000;  /* 假设间隔1ms */
	}
	imu_dt = (imu_tick_now - imu_tick_last) * 0.000001f;  /* us转s */
	imu_tick_last = imu_tick_now;

	/* 防止异常值：如果采样间隔超过1秒，强制设为1ms */
	if(imu_dt > 1)
	{
		imu_dt = 0.001f;
	}

	/* 核心函数：EKF更新四元数，融合陀螺仪和加速度计数据 */
	IMU_QuaternionEKF_Update(gyrox, gyroy, gyroz, accx, accy, accz, imu_dt);

	/* 从EKF结果中提取欧拉角 */
	imu_info->base_info.yaw = QEKF_INS.Yaw;
	imu_info->base_info.pitch = QEKF_INS.Pitch;
	imu_info->base_info.roll = QEKF_INS.Roll;
	imu_info->base_info.yaw_total_angle = QEKF_INS.YawTotalAngle;  /* 累计Yaw角（可超360°） */
#endif

	/* ==================== 第六阶段：后处理 ==================== */

	/* 计算世界坐标系下的加速度（去除重力分量，用于运动控制） */
	pitch = imu_info->base_info.pitch;
	roll = imu_info->base_info.roll;
	yaw = imu_info->base_info.yaw;
	BMI_Get_Acceleration(pitch, roll, yaw,
	                     accx_, accy_, accz_,
	                     &imu_info->base_info.accx, &imu_info->base_info.accy, &imu_info->base_info.accz);

	/* 计算角速度（将rad/s转换为度/s）并进行滑动平均滤波 */
	/* pitch轴角速度 */
	imu_info->base_info.rate_pitch = gyroy_ / (double)0.017453;  /* rad/s转度/s */
	imu_info->base_info.ave_rate_pitch = ave_fil_update(&imu_pitch_dif_speed_ave_filter, imu_info->base_info.rate_pitch, 3);

	/* roll轴角速度 */
	imu_info->base_info.rate_roll = gyrox_ / (double)0.017453;
	imu_info->base_info.ave_rate_roll = ave_fil_update(&imu_roll_dif_speed_ave_filter, imu_info->base_info.rate_roll, 3);

	/* yaw轴角速度 */
	imu_info->base_info.rate_yaw = gyroz_ / (double)0.017453;
	imu_info->base_info.ave_rate_yaw = ave_fil_update(&imu_yaw_dif_speed_ave_filter, imu_info->base_info.rate_yaw, 3);

	/* 重置离线计数器（收到有效数据，设备在线） */
	imu_sen->work_state.offline_cnt = 0;

	/* ==================== 数据有效性检查 ==================== */
	/* 如果三轴加速度和角速度全部为0，说明SPI通信异常或传感器故障 */
	if ((accel[0] == 0) && (accel[1] == 0) && (accel[2] == 0) &&
		 (gyro[0] == 0) && (gyro[1] == 0) && (gyro[2] == 0))
	{
		/* 连续100次数据异常，判定设备离线 */
		if(++imu_sen->work_state.err_cnt >= 100)
		{
			imu_sen->work_state.dev_state = DEV_OFFLINE;
			imu_sen->work_state.err_code = IMU_DATA_ERR;
			imu_sen->work_state.offline_cnt = imu_sen->work_state.offline_max_cnt;
			imu_sen->work_state.err_cnt = 100;  /* 钳位防止溢出 */
		}
	}

	/* 存储IMU芯片温度 */
	imu_info->base_info.temperature = temp;
}

#if IMU_USE_EKF == 1
/**
  * @brief  使用加速度计数据初始化四元数
  * @param  init_q4: 输出参数，存储初始化后的四元数[q0, q1, q2, q3]
  * @retval None
  * @details 原理：静止时加速度计测量的是重力方向，利用重力方向可以确定
  *          Roll和Pitch角，然后转换为四元数。Yaw角置0（磁力计未使用时无法确定Yaw）。
  *
  *          初始化步骤：
  *          1. 读取100次加速度计数据取平均值（消除噪声）
  *          2. 根据重力方向计算Pitch和Roll欧拉角
  *          3. 将欧拉角转换为四元数
  *
  * @note   这种方法避免了初始时刻的姿态估计误差，使EKF能快速收敛
  */
static void InitQuaternion(float *init_q4)
{
    float acc_sum[3] = {0};          /* 加速度累加值 */
	float gyro_init[3], acc_init[3]; /* 临时读取缓冲区 */

    /* 读取100次加速度计数据，取平均值作为初始重力方向估计 */
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_read(gyro_init, acc_init, &temp);  /* 读取传感器数据 */
        acc_sum[0] += acc_init[0];  /* 累加X轴 */
        acc_sum[1] += acc_init[1];  /* 累加Y轴 */
        acc_sum[2] += acc_init[2];  /* 累加Z轴 */
        delay_ms(1);                /* 等待1ms */
    }

    /* 计算平均加速度 */
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] = acc_sum[i] / 100.0f;

    /* 根据重力方向计算欧拉角 */
    float pitch, roll, temp;
    /* Pitch = atan2(-ax, sqrt(ay² + az²)) */
    arm_sqrt_f32(acc_init[1] * acc_init[1] + acc_init[2] * acc_init[2], &temp);
    arm_atan2_f32(-acc_init[0], temp, &pitch);

    /* Roll = atan2(ay, az) */
    arm_atan2_f32(acc_init[1], acc_init[2], &roll);

    /* 将欧拉角转换为四元数 */
    float half_pitch = pitch / 2.0f;  /* 半角 */
    float half_roll = roll / 2.0f;

    /* 四元数计算公式：
     * q0 = cos(pitch/2) * cos(roll/2)
     * q1 = sin(pitch/2) * cos(roll/2)
     * q2 = cos(pitch/2) * sin(roll/2)
     * q3 = sin(pitch/2) * sin(roll/2)
     */
    init_q4[0] = arm_cos_f32(half_pitch) * arm_cos_f32(half_roll);
    init_q4[1] = arm_sin_f32(half_pitch) * arm_cos_f32(half_roll);
    init_q4[2] = arm_cos_f32(half_pitch) * arm_sin_f32(half_roll);
    init_q4[3] = arm_sin_f32(half_pitch) * arm_sin_f32(half_roll);
}

#endif //IMU_USE_EKF
