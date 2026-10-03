/**
 ******************************************************************************
 * @file    kalman_filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   卡尔曼滤波器的C语言实现
 ******************************************************************************
 * @attention
 * 该卡尔曼滤波器可以在传感器采样频率不同的情况下，动态调整矩阵H R和K的维数与数值。
 * This implementation of kalman filter can dynamically adjust dimension and
 * value of matrix H R and K according to the measurement validity under any
 * circumstance that the sampling rate of component sensors are different.
 *
 * 因此矩阵H和R的初始化会与矩阵P A和Q有所不同。另外的，在初始化量测向量z时需要额外写
 * 入传感器量测所对应的状态与这个量测的方式，详情请见例程
 * Therefore, the initialization of matrix P, F, and Q is sometimes different
 * from that of matrices H R. when initialization. Additionally, the corresponding
 * state and the method of the measurement should be provided when initializing
 * measurement vector z. For more details, please see the example.
 *
 * 若不需要动态调整量测向量z，可简单将结构体中的Use_Auto_Adjustment初始化为0，并像初
 * 始化矩阵P那样用常规方式初始化z H R即可。
 * If automatic adjustment is not required, assign zero to the UseAutoAdjustment
 * and initialize z H R in the normal way as matrix P.
 *
 * 要求量测向量z与控制向量u在传感器回调函数中更新。整数0意味着量测无效，即自上次卡尔曼
 * 滤波更新后无传感器数据更新。因此量测向量z与控制向量u会在卡尔曼滤波更新过程中被清零
 * MeasuredVector and ControlVector are required to be updated in the sensor
 * callback function. Integer 0 in measurement vector z indicates the invalidity
 * of current measurement, so MeasuredVector and ControlVector will be reset
 * (to 0) during each update.
 *
 * 此外，矩阵P过度收敛后滤波器将难以再适应状态的缓慢变化，从而产生滤波估计偏差。该算法
 * 通过限制矩阵P最小值的方法，可有效抑制滤波器的过度收敛，详情请见例程。
 * Additionally, the excessive convergence of matrix P will make filter incapable
 * of adopting the slowly changing state. This implementation can effectively
 * suppress filter excessive convergence through boundary limiting for matrix P.
 * For more details, please see the example.
 *
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 设置最小方差
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 开启自动调整
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 设置矩阵值
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 循环更新
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 测量数据更新应按照以下形式 即向MeasuredVector赋值
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */


#include "kalman_filter.h"

/* 内存分配宏定义，若未定义 user_malloc 则默认使用 FreeRTOS 的动态内存分配 */
#ifndef user_malloc
#define user_malloc pvPortMalloc
#endif

/**
 * 卡尔曼滤波器"黄金五式"简述：
 *
 * 预测阶段（时间更新）：
 *   1. 状态先验估计：xhat'(k) = F * xhat(k-1) + B * u
 *   2. 协方差先验估计：P'(k) = F * P(k-1) * F^T + Q
 *
 * 更新阶段（量测更新）：
 *   3. 计算卡尔曼增益：K(k) = P'(k) * H^T / (H * P'(k) * H^T + R)
 *   4. 状态后验估计：xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k))
 *   5. 协方差后验估计：P(k) = (I - K(k) * H) * P'(k)
 */

/* 静态函数前向声明 */
static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
 * @brief  初始化卡尔曼滤波器，分配所有矩阵所需内存并设置维度信息
 * @param  kf:        指向卡尔曼滤波器结构体的指针
 * @param  xhatSize:  状态变量的维度（即状态向量x的元素个数）
 * @param  uSize:     控制变量的维度（若无控制输入则为0）
 * @param  zSize:     观测量的维度（即量测向量z的元素个数）
 * @retval 无
 * @note   本函数为卡尔曼滤波器分配所有必需的矩阵内存，包括：
 *         - 状态向量 xhat（后验估计）和 xhatminus（先验估计）
 *         - 控制向量 u（可选）
 *         - 量测向量 z
 *         - 协方差矩阵 P（后验）和 Pminus（先验）
 *         - 状态转移矩阵 F 及其转置 FT
 *         - 控制矩阵 B（可选）
 *         - 量测矩阵 H 及其转置 HT
 *         - 过程噪声协方差矩阵 Q
 *         - 量测噪声协方差矩阵 R
 *         - 卡尔曼增益矩阵 K
 *         - 临时矩阵和向量（用于中间计算）
 *         所有矩阵初始化为零，用户需在调用本函数后设置具体矩阵值。
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
	/* 记录各向量/矩阵的维度信息 */
	kf->xhatSize = xhatSize;   /* 状态变量维度 */
	kf->uSize = uSize;         /* 控制变量维度 */
	kf->zSize = zSize;         /* 观测量维度 */

	/* 有效量测计数器，用于动态调整时记录当前周期有多少个有效传感器数据 */
	kf->MeasurementValidNum = 0;

	/* ==================== 量测相关辅助数组 ==================== */
	/* MeasurementMap: 量测映射表，记录每个量测值对应的状态变量编号 */
	kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
	memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);

	/* MeasurementDegree: 量测权重/系数，用于构建H矩阵的对应行 */
	kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
	memset(kf->MeasurementDegree, 0, sizeof_float * zSize);

	/* MatR_DiagonalElements: 量测噪声协方差矩阵R的对角线元素 */
	kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
	memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);

	/* StateMinVariance: 各状态变量的最小方差下限，防止P矩阵过度收敛 */
	kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
	memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);

	/* temp: 临时数组，用于动态调整时记录有效量测的原始索引 */
	kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
	memset(kf->temp, 0, sizeof(uint8_t) * zSize);

	/* ==================== 滤波器输入/输出数据 ==================== */
	/* FilteredValue: 滤波输出值，即后验状态估计 xhat(k|k) */
	kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
	memset(kf->FilteredValue, 0, sizeof_float * xhatSize);

	/* MeasuredVector: 外部传感器写入的原始量测值（在回调函数中更新） */
	kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
	memset(kf->MeasuredVector, 0, sizeof_float * zSize);

	/* ControlVector: 外部写入的控制输入向量 */
	kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
	memset(kf->ControlVector, 0, sizeof_float * uSize);

	/* ==================== 状态向量矩阵 ==================== */
	/* xhat: 后验状态估计向量 x(k|k)，维度为 xhatSize x 1 */
	kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
	memset(kf->xhat_data, 0, sizeof_float * xhatSize);
	Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

	/* xhatminus: 先验状态估计向量 x(k|k-1)，维度为 xhatSize x 1 */
	kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
	memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
	Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

	/* ==================== 控制向量矩阵（可选） ==================== */
	if (uSize != 0)
	{
		/* u: 控制输入向量，维度为 uSize x 1 */
		kf->u_data = (float *)user_malloc(sizeof_float * uSize);
		memset(kf->u_data, 0, sizeof_float * uSize);
		Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
	}

	/* ==================== 量测向量矩阵 ==================== */
	/* z: 量测向量，维度为 zSize x 1 */
	kf->z_data = (float *)user_malloc(sizeof_float * zSize);
	memset(kf->z_data, 0, sizeof_float * zSize);
	Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

	/* ==================== 协方差矩阵 ==================== */
	/* P: 后验误差协方差矩阵 P(k|k)，维度为 xhatSize x xhatSize */
	kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

	/* Pminus: 先验误差协方差矩阵 P(k|k-1)，维度为 xhatSize x xhatSize */
	kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

	/* ==================== 状态转移矩阵 ==================== */
	/* F: 状态转移矩阵（系统矩阵），维度为 xhatSize x xhatSize */
	kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	/* FT: F的转置矩阵 */
	kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
	memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
	Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

	/* ==================== 控制矩阵（可选） ==================== */
	if (uSize != 0)
	{
		/* B: 控制输入矩阵，维度为 xhatSize x uSize */
		kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
		memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
		Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
	}

	/* ==================== 量测矩阵 ==================== */
	/* H: 量测矩阵，维度为 zSize x xhatSize */
	kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
	/* HT: H的转置矩阵，维度为 xhatSize x zSize */
	kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
	memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
	memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
	Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
	Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

	/* ==================== 噪声协方差矩阵 ==================== */
	/* Q: 过程噪声协方差矩阵，维度为 xhatSize x xhatSize */
	kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
	memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
	Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

	/* R: 量测噪声协方差矩阵，维度为 zSize x zSize */
	kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
	memset(kf->R_data, 0, sizeof_float * zSize * zSize);
	Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

	/* ==================== 卡尔曼增益矩阵 ==================== */
	/* K: 卡尔曼增益矩阵，维度为 xhatSize x zSize */
	kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
	memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
	Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

	/* ==================== 临时矩阵/向量（用于中间计算） ==================== */
	/* S: 新息协方差矩阵 S = H * P' * H^T + R，维度为 xhatSize x xhatSize */
	kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
	/* 临时矩阵，用于存放中间计算结果 */
	kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
	kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
	/* 临时向量，用于存放中间计算结果 */
	kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
	kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);

	Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
	Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
	Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
	Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
	Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

	/* ==================== 跳过标志位 ==================== */
	/* SkipEq1~5: 用于跳过"黄金五式"中某个步骤的标志位，方便扩展为EKF/UKF等 */
	kf->SkipEq1 = 0;
	kf->SkipEq2 = 0;
	kf->SkipEq3 = 0;
	kf->SkipEq4 = 0;
	kf->SkipEq5 = 0;
}

/**
 * @brief  获取量测数据，根据是否开启自动调整来处理量测向量z和控制向量u
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval 无
 * @note   - 若开启自动调整（UseAutoAdjustment != 0），调用 H_K_R_Adjustment
 *           根据当前有效量测动态重构 H、R、K 矩阵和 z 向量。
 *         - 若未开启自动调整，则直接将 MeasuredVector 复制到 z_data，
 *           并清零 MeasuredVector 以便下次更新时检测新数据。
 *         - 无论哪种模式，都会将 ControlVector 复制到 u_data。
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
	/* 根据量测情况自动调整矩阵 H、K、R 的维数和数值 */
	if (kf->UseAutoAdjustment != 0)
		H_K_R_Adjustment(kf);  /* 动态调整模式 */
	else
	{
		/* 非自动调整模式：直接复制量测数据到z向量，然后清零原始量测缓冲区 */
		memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
		memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
	}

	/* 将控制输入向量复制到滤波器内部的 u 向量 */
	memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

/**
 * @brief  卡尔曼滤波第一式：状态先验估计（预测步）
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval 无
 * @note   计算公式：xhat'(k) = F * xhat(k-1) + B * u
 *         若无控制输入（uSize == 0），则简化为：xhat'(k) = F * xhat(k-1)
 *         SkipEq1 标志位为1时跳过此步骤。
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
	/* 检查是否跳过此步骤 */
	if (!kf->SkipEq1)
	{
		if (kf->uSize > 0)
		{
			/*
			 * 有控制输入的情况：
			 * temp_vector = F * xhat(k-1)        （状态转移预测）
			 * temp_vector1 = B * u(k)             （控制输入贡献）
			 * xhatminus = temp_vector + temp_vector1 （叠加得到先验估计）
			 */
			kf->temp_vector.numRows = kf->xhatSize;
			kf->temp_vector.numCols = 1;
			kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);

			kf->temp_vector1.numRows = kf->xhatSize;
			kf->temp_vector1.numCols = 1;
			kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);

			kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
		}
		else
		{
			/* 无控制输入：xhatminus = F * xhat(k-1) */
			kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
		}
	}
}

/**
 * @brief  卡尔曼滤波第二式：协方差先验估计（预测步）
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval 无
 * @note   计算公式：P'(k) = F * P(k-1) * F^T + Q
 *         SkipEq2 标志位为1时跳过此步骤。
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
	/* 检查是否跳过此步骤 */
	if (!kf->SkipEq2)
	{
		/* 计算F的转置：FT = F^T */
		kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);

		/* 计算中间结果：Pminus = F * P(k-1) */
		kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);

		/* 计算 temp_matrix = Pminus * FT = F * P(k-1) * F^T */
		kf->temp_matrix.numRows = kf->Pminus.numRows;
		kf->temp_matrix.numCols = kf->FT.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix);

		/* 最终结果：Pminus = F * P(k-1) * F^T + Q */
		kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
	}
}

/**
 * @brief  卡尔曼滤波第三式：计算卡尔曼增益（更新步）
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval 无
 * @note   计算公式：K(k) = P'(k) * H^T * inv(H * P'(k) * H^T + R)
 *         SkipEq3 标志位为1时跳过此步骤。
 *         注意：此步骤中矩阵H、R、K的维数可能在自动调整模式下已发生变化。
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
	/* 检查是否跳过此步骤 */
	if (!kf->SkipEq3)
	{
		/* 计算H的转置：HT = H^T */
		kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);

		/* 计算 temp_matrix = H * P'(k) */
		kf->temp_matrix.numRows = kf->H.numRows;
		kf->temp_matrix.numCols = kf->Pminus.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);

		/* 计算 temp_matrix1 = H * P'(k) * H^T */
		kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
		kf->temp_matrix1.numCols = kf->HT.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);

		/* 计算 S = H * P'(k) * H^T + R（新息协方差矩阵） */
		kf->S.numRows = kf->R.numRows;
		kf->S.numCols = kf->R.numCols;
		kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);

		/* 计算 S 的逆矩阵：temp_matrix1 = inv(S) */
		kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

		/* 计算 temp_matrix = P'(k) * H^T */
		kf->temp_matrix.numRows = kf->Pminus.numRows;
		kf->temp_matrix.numCols = kf->HT.numCols;
		kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);

		/* 最终结果：K = P'(k) * H^T * inv(S) */
		kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
	}
}

/**
 * @brief  卡尔曼滤波第四式：状态后验估计（融合步）
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval 无
 * @note   计算公式：xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k))
 *         SkipEq4 标志位为1时跳过此步骤。
 *         这是卡尔曼滤波的核心融合步骤，将先验估计与量测数据结合。
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
	/* 检查是否跳过此步骤 */
	if (!kf->SkipEq4)
	{
		/* 计算 temp_vector = H * xhat'(k) （先验估计的量测预测） */
		kf->temp_vector.numRows = kf->H.numRows;
		kf->temp_vector.numCols = 1;
		kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector);

		/* 计算 temp_vector1 = z(k) - H * xhat'(k) （量测残差/新息） */
		kf->temp_vector1.numRows = kf->z.numRows;
		kf->temp_vector1.numCols = 1;
		kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

		/* 计算 temp_vector = K(k) * (z(k) - H * xhat'(k)) （增益加权残差） */
		kf->temp_vector.numRows = kf->K.numRows;
		kf->temp_vector.numCols = 1;
		kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

		/* 最终结果：xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k)) */
		kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
	}
}

/**
 * @brief  卡尔曼滤波第五式：协方差后验估计（更新步）
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval 无
 * @note   计算公式：P(k) = P'(k) - K(k) * H * P'(k)
 *         等价于：P(k) = (I - K(k) * H) * P'(k)
 *         SkipEq5 标志位为1时跳过此步骤。
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
	/* 检查是否跳过此步骤 */
	if (!kf->SkipEq5)
	{
		/* 计算 temp_matrix = K(k) * H */
		kf->temp_matrix.numRows = kf->K.numRows;
		kf->temp_matrix.numCols = kf->H.numCols;
		kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
		kf->temp_matrix1.numCols = kf->Pminus.numCols;

		kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);

		/* 计算 temp_matrix1 = K(k) * H * P'(k) */
		kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1);

		/* 最终结果：P(k) = P'(k) - K(k) * H * P'(k) */
		kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
	}
}

/**
 * @brief  执行卡尔曼滤波完整更新流程（"黄金五式"）
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval float* 返回滤波后的状态估计值数组指针（即 FilteredValue）
 * @note   本函数是卡尔曼滤波器的核心入口函数，按顺序执行以下步骤：
 *         步骤0：获取量测数据（Measurement -> z, u）
 *         步骤1：状态先验估计  xhat'(k) = F * xhat(k-1) + B * u
 *         步骤2：协方差先验估计 P'(k) = F * P(k-1) * F^T + Q
 *         步骤3：计算卡尔曼增益 K(k) = P'(k) * H^T * inv(H * P'(k) * H^T + R)
 *         步骤4：状态后验估计 xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k))
 *         步骤5：协方差后验估计 P(k) = P'(k) - K(k) * H * P'(k)
 *
 *         每个步骤后可调用用户自定义函数（User_Func0_f ~ User_Func6_f），
 *         方便扩展为扩展卡尔曼滤波（EKF）、无迹卡尔曼滤波（UKF）等变体。
 *
 *         若当前无有效量测数据（MeasurementValidNum == 0），则跳过更新步骤3~5，
 *         仅执行预测，此时 xhat(k) = xhat'(k), P(k) = P'(k)。
 *
 *         最后限制 P 矩阵对角线元素不低于最小方差，防止滤波器过度收敛。
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
	/* ========== 步骤0：获取量测信息 ========== */
	Kalman_Filter_Measure(kf);  /* 处理量测向量z和控制向量u */
	if (kf->User_Func0_f != NULL)
		kf->User_Func0_f(kf);  /* 用户自定义回调（量测预处理） */

	/* ========== 步骤1：状态先验估计（预测） ========== */
	/* xhat'(k) = A * xhat(k-1) + B * u */
	Kalman_Filter_xhatMinusUpdate(kf);
	if (kf->User_Func1_f != NULL)
		kf->User_Func1_f(kf);  /* 用户自定义回调 */

	/* ========== 步骤2：协方差先验估计（预测） ========== */
	/* P'(k) = A * P(k-1) * AT + Q */
	Kalman_Filter_PminusUpdate(kf);
	if (kf->User_Func2_f != NULL)
		kf->User_Func2_f(kf);  /* 用户自定义回调 */

	/* ========== 步骤3/4/5：量测更新（仅在有有效量测时执行） ========== */
	if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
	{
		/* 步骤3：计算卡尔曼增益 K(k) = P'(k) * HT / (H * P'(k) * HT + R) */
		Kalman_Filter_SetK(kf);
		if (kf->User_Func3_f != NULL)
			kf->User_Func3_f(kf);  /* 用户自定义回调 */

		/* 步骤4：融合量测数据更新状态估计 */
		/* xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k)) */
		Kalman_Filter_xhatUpdate(kf);
		if (kf->User_Func4_f != NULL)
			kf->User_Func4_f(kf);  /* 用户自定义回调 */

		/* 步骤5：修正协方差矩阵 */
		/* P(k) = P'(k) - K(k) * H * P'(k) */
		Kalman_Filter_P_Update(kf);
	}
	else
	{
		/*
		 * 无有效量测数据，仅执行预测，不做量测更新：
		 * xhat(k) = xhat'(k)
		 * P(k) = P'(k)
		 */
		memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
		memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
	}

	/* 用户自定义回调（可用于后处理等） */
	if (kf->User_Func5_f != NULL)
		kf->User_Func5_f(kf);

	/*
	 * 防止滤波器过度收敛：
	 * 当P矩阵对角线元素（即各状态的估计方差）小于设定的最小方差阈值时，
	 * 将其强制设为最小值。这样可以避免滤波器对缓慢变化的状态产生估计偏差。
	 */
	for (uint8_t i = 0; i < kf->xhatSize; ++i)
	{
		if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
			kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
	}

	/* 将滤波结果复制到 FilteredValue 供外部读取 */
	memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

	/* 用户自定义回调（可用于输出后处理等） */
	if (kf->User_Func6_f != NULL)
		kf->User_Func6_f(kf);

	return kf->FilteredValue;  /* 返回滤波后的状态估计值 */
}

/**
 * @brief  �态调整量测矩阵H、噪声协方差矩阵R和卡尔曼增益矩阵K的维数与数值
 * @param  kf: 指向卡尔曼滤波器结构体的指针
 * @retval 无
 * @note   本函数在自动调整模式（UseAutoAdjustment != 0）下被调用。
 *         它根据当前周期各传感器量测的有效性（z_data[i] != 0 视为有效），
 *         动态重构 z 向量、H 矩阵和 R 矩阵。
 *
 *         处理流程：
 *         1. 将 MeasuredVector 复制到 z_data，并清零 MeasuredVector
 *         2. 遍历 z_data，识别有效量测（非零值），重新排列 z_data
 *         3. 根据 MeasurementMap 和 MeasurementDegree 重构 H 矩阵对应行
 *         4. 根据有效量测对应的 MatR_DiagonalElements 重构 R 矩阵对角线
 *         5. 调整 H、HT、R、K、z 矩阵的维数以匹配当前有效量测数量
 *
 *         这种机制允许不同传感器以不同频率采样，无需额外的同步逻辑。
 */
static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
	/* 重置有效量测计数器 */
	kf->MeasurementValidNum = 0;

	/* 将外部量测数据复制到内部 z_data，并清零外部缓冲区 */
	memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
	memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

	/*
	 * 第一步：识别有效量测并重构 z 向量和 H 矩阵
	 * - 有效量测的判断条件：z_data[i] != 0
	 * - 将有效量测重新排列到 z_data 的前面
	 * - 同时记录有效量测的原始索引到 temp 数组（用于重构R矩阵）
	 */
	memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);   /* 清零R矩阵 */
	memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize); /* 清零H矩阵 */

	for (uint8_t i = 0; i < kf->zSize; ++i)
	{
		if (kf->z_data[i] != 0)
		{
			/* 重构向量z：将有效量测值前移排列 */
			kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
			/* 记录原始索引，以便后续从预设数组中取对应的R值 */
			kf->temp[kf->MeasurementValidNum] = i;

			/*
			 * 重构矩阵H：
			 * MeasurementMap[i] - 1 表示该量测对应的状态变量在状态向量中的索引（从0开始）
			 * MeasurementDegree[i] 是该量测的权重系数
			 * H矩阵第 MeasurementValidNum 行、第 (MeasurementMap[i]-1) 列 设为 MeasurementDegree[i]
			 */
			kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];

			kf->MeasurementValidNum++;  /* 有效量测计数加1 */
		}
	}

	/*
	 * 第二步：重构矩阵R
	 * R矩阵为对角矩阵，只设置对角线元素
	 * 从预设的 MatR_DiagonalElements 数组中取出对应量测的噪声方差值
	 */
	for (uint8_t i = 0; i < kf->MeasurementValidNum; ++i)
	{
		kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
	}

	/*
	 * 第三步：调整矩阵维数
	 * 由于有效量测数量可能少于 zSize，需要将 H、HT、R、K、z 的维数
	 * 调整为与当前有效量测数量匹配，以保证矩阵运算的正确性
	 */
	kf->H.numRows = kf->MeasurementValidNum;   /* H的行数 = 有效量测数 */
	kf->H.numCols = kf->xhatSize;              /* H的列数 = 状态变量数 */
	kf->HT.numRows = kf->xhatSize;             /* HT的行数 = 状态变量数 */
	kf->HT.numCols = kf->MeasurementValidNum;  /* HT的列数 = 有效量测数 */
	kf->R.numRows = kf->MeasurementValidNum;   /* R的行数 = 有效量测数 */
	kf->R.numCols = kf->MeasurementValidNum;   /* R的列数 = 有效量测数 */
	kf->K.numRows = kf->xhatSize;              /* K的行数 = 状态变量数 */
	kf->K.numCols = kf->MeasurementValidNum;   /* K的列数 = 有效量测数 */
	kf->z.numRows = kf->MeasurementValidNum;   /* z的行数 = 有效量测数 */
}
