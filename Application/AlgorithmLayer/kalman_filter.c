/**
  ******************************************************************************
  * @file    : kalman_filter.c
  * @author  : Wang Hongxi
  * @version : V1.2.2
  * @date    : 2022/1/8
  * @brief   : 卡尔曼滤波器的C语言实现
  *
  * @details : 该实现支持多维状态空间的卡尔曼滤波，适用于传感器融合、状态估计等场景。
  *            主要特性：
  *            1. 支持动态调整量测矩阵H、噪声协方差矩阵R和卡尔曼增益K的维数
  *            2. 适用于不同传感器采样频率不同的情况
  *            3. 通过限制矩阵P的最小值防止滤波器过度收敛
  *            4. 支持自定义函数扩展（可扩展为EKF/UKF/ESKF等）
  *
  * @attention:
  *            - 量测向量z中的整数0表示当前量测无效
  *            - 量测向量z和控制向量u需在传感器回调函数中更新
  *            - 矩阵P过度收敛会导致滤波器无法适应状态的缓慢变化
  *
  * @example  : 以高度估计为例
  *   状态向量 x = | height |    量测来源：气压计、GPS、加速度计
  *                | velocity |
  *                |acceleration|
  *
  *   初始化示例：
  *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);  // 3维状态，0维控制，3维量测
  *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
  *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
  *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
  *
  *   使用示例：
  *     Kalman_Filter_Update(&Height_KF);  // 执行一次卡尔曼滤波更新
  *     float *result = Height_KF.FilteredValue;  // 获取滤波结果
  ******************************************************************************
  */


/* 包含头文件 ------------------------------------------------------------------*/
#include "kalman_filter.h"  /* 卡尔曼滤波器头文件，包含结构体定义和矩阵运算接口 */

/* 内存分配宏定义 --------------------------------------------------------------*/
/**
 * @brief 用户内存分配函数
 * @note  默认使用FreeRTOS的pvPortMalloc，可根据实际系统替换
 */
#ifndef user_malloc
#define user_malloc pvPortMalloc
#endif


/* 内部函数声明 ----------------------------------------------------------------*/
static void H_K_R_Adjustment(KalmanFilter_t *kf);


/**
  * @brief  初始化卡尔曼滤波器，分配所有矩阵所需内存
  * @details 完成以下初始化工作：
  *          1. 设置状态变量、控制变量、观测量的维度
  *          2. 分配量测标志、量测权重、R对角元素、状态最小方差等辅助数组
  *          3. 分配滤波数据缓冲区（滤波值、量测向量、控制向量）
  *          4. 分配并初始化所有卡尔曼滤波所需的矩阵：
  *             - xhat: 状态估计向量 x(k|k)
  *             - xhatminus: 先验状态估计向量 x(k|k-1)
  *             - P: 协方差矩阵 P(k|k)
  *             - Pminus: 先验协方差矩阵 P(k|k-1)
  *             - F: 状态转移矩阵
  *             - FT: 状态转移矩阵的转置
  *             - B: 控制矩阵（可选）
  *             - H: 量测矩阵
  *             - HT: 量测矩阵的转置
  *             - Q: 过程噪声协方差矩阵
  *             - R: 量测噪声协方差矩阵
  *             - K: 卡尔曼增益矩阵
  *             - S: 临时矩阵（用于K的计算）
  *          5. 初始化所有跳过标志为0（不跳过任何方程）
  *
  * @param  kf:       指向卡尔曼滤波器结构体的指针
  * @param  xhatSize: 状态变量维度（如：高度、速度、加速度 = 3维）
  * @param  uSize:    控制变量维度（无控制输入时为0）
  * @param  zSize:    观测量维度（如：气压计+GPS+加速度计 = 3维）
  * @retval None
  */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    /* 记录各向量维度 */
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    /* 初始化有效量测计数为0 */
    kf->MeasurementValidNum = 0;

    /* ===== 分配量测相关辅助数组 ===== */
    /* MeasurementMap: 量测索引映射，将量测向量的索引映射到状态向量的索引 */
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);

    /* MeasurementDegree: 量测权重，表示量测对状态的影响程度 */
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);

    /* MatR_DiagonalElements: 量测噪声协方差矩阵R的对角线元素 */
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);

    /* StateMinVariance: 状态最小方差，用于防止滤波器过度收敛 */
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);

    /* temp: 临时数组，用于量测有效性判断 */
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    /* ===== 分配滤波数据缓冲区 ===== */
    /* FilteredValue: 滤波输出结果，与状态向量同维度 */
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);

    /* MeasuredVector: 原始量测向量，由传感器回调函数填充 */
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);

    /* ControlVector: 控制向量，由控制算法填充 */
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    /* ===== 分配卡尔曼滤波矩阵 ===== */

    /* xhat: 状态估计向量 x(k|k)，维度 [xhatSize x 1] */
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    /* xhatminus: 先验状态估计向量 x(k|k-1)，维度 [xhatSize x 1] */
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    /* 控制向量 u，仅当uSize > 0时分配 */
    if (uSize != 0)
    {
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    /* z: 量测向量，维度 [zSize x 1] */
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    /* P: 协方差矩阵 P(k|k)，维度 [xhatSize x xhatSize] */
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    /* Pminus: 先验协方差矩阵 P(k|k-1)，维度 [xhatSize x xhatSize] */
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    /* F: 状态转移矩阵，维度 [xhatSize x xhatSize] */
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    /* FT: 状态转移矩阵的转置 */
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    /* B: 控制矩阵，仅当uSize > 0时分配，维度 [xhatSize x uSize] */
    if (uSize != 0)
    {
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    /* H: 量测矩阵，维度 [zSize x xhatSize] */
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    /* HT: 量测矩阵的转置 */
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    /* Q: 过程噪声协方差矩阵，维度 [xhatSize x xhatSize] */
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    /* R: 量测噪声协方差矩阵，维度 [zSize x zSize] */
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    /* K: 卡尔曼增益矩阵，维度 [xhatSize x zSize] */
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    /* ===== 分配临时矩阵和向量 ===== */
    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    /* 初始化所有方程跳过标志为0（不跳过任何方程） */
    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
  * @brief  获取量测数据并更新量测向量z和控制向量u
  * @details 将外部传感器数据复制到内部量测向量z，并清空外部量测缓冲区
  *          同时将控制向量u复制到内部缓冲区
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval None
  */
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    /* 将外部量测数据复制到内部z向量 */
    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    /* 清空外部量测缓冲区，标记量测已被消费 */
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    /* 将控制向量复制到内部缓冲区 */
    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

/**
  * @brief  卡尔曼滤波第1步：先验状态估计
  * @details 计算公式：xhat'(k) = F * xhat(k-1) + B * u
  *          即：先验状态 = 状态转移矩阵 * 上次后验状态 + 控制矩阵 * 控制输入
  *
  *          如果有控制输入(uSize > 0)：
  *            xhat' = F * xhat + B * u
  *          如果无控制输入：
  *            xhat' = F * xhat
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval None
  */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    /* 检查是否跳过此方程 */
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            /* 有控制输入：xhat' = F * xhat + B * u */
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);  /* temp = F * xhat */

            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);   /* temp1 = B * u */

            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);  /* xhat' = temp + temp1 */
        }
        else
        {
            /* 无控制输入：xhat' = F * xhat */
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

/**
  * @brief  卡尔曼滤波第2步：先验协方差矩阵更新
  * @details 计算公式：P'(k) = F * P(k-1) * F^T + Q
  *          即：先验协方差 = 状态转移矩阵 * 上次后验协方差 * 转置 + 过程噪声协方差
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval None
  */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        /* 计算F的转置：FT = F^T */
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);

        /* 计算Pminus = F * P */
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);

        /* 计算temp = Pminus * FT = F * P * F^T */
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix);

        /* 计算Pminus = temp + Q = F * P * F^T + Q */
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}

/**
  * @brief  卡尔曼滤波第3步：计算卡尔曼增益K
  * @details 计算公式：K(k) = P'(k) * H^T / (H * P'(k) * H^T + R)
  *          即：卡尔曼增益 = 先验协方差 * 量测矩阵转置 / (量测矩阵 * 先验协方差 * 量测矩阵转置 + 量测噪声)
  *
  *          实际计算使用矩阵求逆：
  *          S = H * P'(k) * H^T + R
  *          K = P'(k) * H^T * inv(S)
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval None
  */
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        /* 计算H的转置：HT = H^T */
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);

        /* 计算temp = H * P'(k) */
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);

        /* 计算temp1 = H * P'(k) * H^T */
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);

        /* 计算S = H * P'(k) * H^T + R（新息协方差） */
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);

        /* 计算temp1 = inv(S) = inv(H * P'(k) * H^T + R) */
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

        /* 计算temp = P'(k) * H^T */
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);

        /* 计算K = P'(k) * H^T * inv(S)（卡尔曼增益） */
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}

/**
  * @brief  卡尔曼滤波第4步：后验状态估计（状态融合）
  * @details 计算公式：xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k))
  *          即：后验状态 = 先验状态 + 卡尔曼增益 * (实际量测 - 预测量测)
  *
  *          这是卡尔曼滤波的核心步骤，将预测值与实际量测值进行融合
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval None
  */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        /* 计算temp = H * xhat'(k)（预测量测） */
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector);

        /* 计算temp1 = z(k) - H * xhat'(k)（新息/量测残差） */
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

        /* 计算temp = K(k) * (z(k) - H * xhat'(k))（卡尔曼增益修正量） */
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

        /* 计算xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k))（后验状态） */
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}

/**
  * @brief  卡尔曼滤波第5步：后验协方差矩阵更新
  * @details 计算公式：P(k) = (I - K(k) * H) * P'(k)
  *          等价于：P(k) = P'(k) - K(k) * H * P'(k)
  *          即：后验协方差 = 先验协方差 - 卡尔曼增益 * 量测矩阵 * 先验协方差
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval None
  */
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        /* 计算temp = K(k) * H */
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);

        /* 计算temp1 = K(k) * H * P'(k) */
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1);

        /* 计算P(k) = P'(k) - K(k) * H * P'(k)（后验协方差） */
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
  * @brief  执行一次完整的卡尔曼滤波更新（黄金五式）
  * @details 卡尔曼滤波的完整流程：
  *          0. 获取量测数据
  *          1. 先验状态估计：xhat'(k) = F * xhat(k-1) + B * u
  *          2. 先验协方差更新：P'(k) = F * P(k-1) * F^T + Q
  *          3. 计算卡尔曼增益：K(k) = P'(k) * H^T * inv(H * P'(k) * H^T + R)
  *          4. 状态融合：xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k))
  *          5. 后验协方差更新：P(k) = P'(k) - K(k) * H * P'(k)
  *
  *          每个步骤都支持通过User_FuncX_f回调函数进行自定义扩展
  *          可用于实现EKF、UKF、ESKF等变体
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval float*: 返回指向滤波结果数组的指针
  */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    /* ===== 步骤0: 获取量测信息 ===== */
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);  /* 用户自定义前处理回调 */

    /* ===== 步骤1: 先验状态估计 ===== */
    /* xhat'(k) = F * xhat(k-1) + B * u */
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);  /* 用户自定义回调 */

    /* ===== 步骤2: 预测更新（先验协方差） ===== */
    /* P'(k) = F * P(k-1) * F^T + Q */
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);  /* 用户自定义回调 */

    /* 判断是否有有效量测数据 */
    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        /* ===== 步骤3: 量测更新（计算卡尔曼增益） ===== */
        /* K(k) = P'(k) * H^T * inv(H * P'(k) * H^T + R) */
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);  /* 用户自定义回调 */

        /* ===== 步骤4: 状态融合 ===== */
        /* xhat(k) = xhat'(k) + K(k) * (z(k) - H * xhat'(k)) */
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);  /* 用户自定义回调 */

        /* ===== 步骤5: 修正协方差矩阵 ===== */
        /* P(k) = P'(k) - K(k) * H * P'(k) */
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        /* 无有效量测数据时，仅使用预测值，不进行量测更新 */
        /* xhat(k) = xhat'(k) */
        /* P(k) = P'(k) */
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    /* 用户自定义后处理回调 */
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    /* ===== 防止滤波器过度收敛 ===== */
    /* 如果P矩阵对角线元素小于设定的最小方差，则将其限制为最小方差 */
    /* 这可以防止滤波器过度自信，使其能适应状态的缓慢变化 */
    for (uint8_t i = 0; i < kf->xhatSize; ++i)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    /* 将滤波结果复制到输出缓冲区 */
    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    /* 用户自定义最终回调 */
    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

/**
  * @brief  重构矩阵z、H、R的数据，并调整矩阵维度
  * @details 根据当前周期的量测有效性，动态调整卡尔曼滤波器的量测相关矩阵：
  *          1. 识别哪些量测数据有效（非零）
  *          2. 重构量测向量z（只保留有效量测）
  *          3. 重构量测矩阵H（只保留有效量测对应的行）
  *          4. 重构量测噪声矩阵R（只保留有效量测对应的对角元素）
  *          5. 调整所有相关矩阵的维度
  *
  *          该功能允许不同传感器以不同频率更新，系统只使用当前周期内有效的量测数据
  *
  * @param  kf: 指向卡尔曼滤波器结构体的指针
  * @retval None
  */
static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    /* 重置有效量测计数 */
    kf->MeasurementValidNum = 0;

    /* 将外部量测数据复制到内部z向量，并清空外部缓冲区 */
    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    /* 清空R矩阵和H矩阵的数据 */
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);

    /* 遍历所有量测数据，识别有效量测并重构z和H矩阵 */
    for (uint8_t i = 0; i < kf->zSize; ++i)
    {
        if (kf->z_data[i] != 0)  /* 非零值表示该量测有效 */
        {
            /* 重构量测向量z：将有效量测紧凑排列 */
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            /* 记录原始量测索引，用于后续重构R矩阵 */
            kf->temp[kf->MeasurementValidNum] = i;

            /* 重构量测矩阵H：
             * MeasurementMap[i]-1: 该量测对应的状态变量索引
             * MeasurementDegree[i]: 量测权重 */
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];

            kf->MeasurementValidNum++;  /* 有效量测计数加1 */
        }
    }

    /* 重构量测噪声协方差矩阵R（仅对角线元素） */
    for (uint8_t i = 0; i < kf->MeasurementValidNum; ++i)
    {
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    /* 调整相关矩阵的维度以匹配有效量测数量 */
    kf->H.numRows = kf->MeasurementValidNum;   /* H矩阵行数 = 有效量测数 */
    kf->H.numCols = kf->xhatSize;              /* H矩阵列数 = 状态维度 */
    kf->HT.numRows = kf->xhatSize;             /* HT矩阵行数 = 状态维度 */
    kf->HT.numCols = kf->MeasurementValidNum;  /* HT矩阵列数 = 有效量测数 */
    kf->R.numRows = kf->MeasurementValidNum;   /* R矩阵行数 = 有效量测数 */
    kf->R.numCols = kf->MeasurementValidNum;   /* R矩阵列数 = 有效量测数 */
    kf->K.numRows = kf->xhatSize;              /* K矩阵行数 = 状态维度 */
    kf->K.numCols = kf->MeasurementValidNum;   /* K矩阵列数 = 有效量测数 */
    kf->z.numRows = kf->MeasurementValidNum;   /* z向量行数 = 有效量测数 */
}
