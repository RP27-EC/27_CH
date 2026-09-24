#ifndef __CONFIG_CHASSIS_H
#define __CONFIG_CHASSIS_H

#define OFF_GROUND_TEST 0

#define TIME_STEP			0.001f//任务运行周期，单位：s

#define WHEEL_RADIUS  0.0515f//驱动轮半径，单位：m

#define WHEEL_REDUCT_RATIO        (7.f/1.f)//电机到轮毂的减速比
//整车长度
#define Car_Length 0.338f
//整车宽度
#define Car_Width 0.3f
//整车旋转半径
#define Rl 0.2259668f
//整车旋转夹角（0~PI/2)
#define theta_Rl 0.7259073169f
//车体运动最大速度
#define MAX_SPEED           2.5f    //单位：m/s
//车体转向运动最大速度
#define MAX_SPIN_SPEED           13.0f    //单位：rad/s
//车体中心离地高度
#define GRAVITY_HIGHT           0.18f    //单位：m
//全车重量
#define CAR_GRAVITY           196.f    //单位：N

/*电机方向与归位相关*/
//舵向电机的零点
#define L_F_ZeroPoint    32768
#define L_B_ZeroPoint    32768
#define R_B_ZeroPoint    32768
#define R_F_ZeroPoint    32768

//航向电机正方向
#define L_F_Direction   1
#define R_F_Direction   -1
#define R_B_Direction   -1
#define L_B_Direction   1

/*卸力阻尼时间与阻尼系数*/
#define DAMPING_DELAY_MAX_CNT     3000   //阻尼持续时间2.5s
#define Wheel_Damping_Coefficient 0.005f //
#define Sd_Damping_Coefficient    0.002f

#define SD_POS_FIX_TOR_K			(30.f)   //关节限位力矩补偿系数 10度1N

#define DISTANCE_ERR_MAX      0.55f


#endif
