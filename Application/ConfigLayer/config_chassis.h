/**
  ******************************************************************************
  * @file           : config_chassis.h
  * @brief          : 底盘配置参数头文件
  * @details        : 定义底盘的物理参数、运动限制、电机配置等关键常量。
  *                   这些参数需要根据实际机械结构进行调整。
  ******************************************************************************
  */

#ifndef __CONFIG_CHASSIS_H
#define __CONFIG_CHASSIS_H

/**
  * @brief  离地测试模式开关
  * @details 0=正常模式，1=离地测试模式（用于调试时车轮离地运行）
  */
#define OFF_GROUND_TEST 0

/**
  * @brief  控制任务运行周期
  * @details 单位：秒(s)，0.001s = 1ms
  */
#define TIME_STEP			0.001f

/**
  * @brief  驱动轮半径
  * @details 单位：米(m)，M3508电机配套的麦轮半径
  */
#define WHEEL_RADIUS  0.0515f

/**
  * @brief  电机到轮毂的减速比
  * @details M3508电机内置减速器，减速比为7:1
  *          即电机转7圈，轮子转1圈
  */
#define WHEEL_REDUCT_RATIO        (7.f/1.f)

/**
  * @brief  整车长度
  * @details 单位：米(m)，车体前后轮距
  */
#define Car_Length 0.338f

/**
  * @brief  整车宽度
  * @details 单位：米(m)，车体左右轮距
  */
#define Car_Width 0.3f

/**
  * @brief  整车旋转半径
  * @details 单位：米(m)，车体绕中心旋转时的等效半径
  *          用于计算旋转时各轮的速度差异
  */
#define Rl 0.2259668f

/**
  * @brief  整车旋转夹角
  * @details 单位：弧度(rad)，范围[0, π/2]
  *          轮子安装角度，影响麦轮的运动学特性
  */
#define theta_Rl 0.7259073169f

/**
  * @brief  车体运动最大速度
  * @details 单位：米/秒(m/s)，底盘平移运动的速度上限
  */
#define MAX_SPEED           2.5f

/**
  * @brief  车体转向运动最大速度
  * @details 单位：弧度/秒(rad/s)，底盘旋转运动的角速度上限
  */
#define MAX_SPIN_SPEED           13.0f

/**
  * @brief  车体中心离地高度
  * @details 单位：米(m)，用于坡度检测和姿态计算
  */
#define GRAVITY_HIGHT           0.18f

/**
  * @brief  全车重量
  * @details 单位：牛顿(N)，约20kg（196N / 9.8m/s²）
  */
#define CAR_GRAVITY           196.f

/*============================== 电机方向与归位配置 ==============================*/

/**
  * @brief  舵向电机零点
  * @details 舵向电机的机械零点位置，用于上电归位
  *          32768对应16位编码器的中间位置
  */
#define L_F_ZeroPoint    32768  /* 左前舵向电机零点 */
#define L_B_ZeroPoint    32768  /* 左后舵向电机零点 */
#define R_B_ZeroPoint    32768  /* 右后舵向电机零点 */
#define R_F_ZeroPoint    32768  /* 右前舵向电机零点 */

/**
  * @brief  航向电机正方向
  * @details 定义每个舵向电机的正方向，用于补偿安装差异
  *          1=正向，-1=反向
  */
#define L_F_Direction   1    /* 左前舵向电机正方向 */
#define R_F_Direction   -1   /* 右前舵向电机正方向 */
#define R_B_Direction   -1   /* 右后舵向电机正方向 */
#define L_B_Direction   1    /* 左后舵向电机正方向 */

/*============================== 阻尼与补偿参数 ==============================*/

/**
  * @brief  阻尼持续时间
  * @details 单位：毫秒(ms)，3000ms = 3s
  *          用于卸力时的阻尼衰减时间
  */
#define DAMPING_DELAY_MAX_CNT     3000

/**
  * @brief  轮子阻尼系数
  * @details 卸力时轮子的阻尼衰减系数，值越小衰减越慢
  */
#define Wheel_Damping_Coefficient 0.005f

/**
  * @brief  舵向阻尼系数
  * @details 卸力时舵向电机的阻尼衰减系数
  */
#define Sd_Damping_Coefficient    0.002f

/**
  * @brief  关节限位力矩补偿系数
  * @details 单位：N/度，10度对应1N的补偿力矩
  *          用于舵向电机在限位位置的力矩补偿
  */
#define SD_POS_FIX_TOR_K			(30.f)

/**
  * @brief  最大距离误差
  * @details 单位：米(m)，用于位置控制时的误差限幅
  */
#define DISTANCE_ERR_MAX      0.55f


#endif
