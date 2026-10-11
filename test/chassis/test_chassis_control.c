/**
  ******************************************************************************
  * @file    test_chassis_control.c
  * @brief   底盘控制单元测试文件
  * @details 本文件包含底盘控制逻辑的单元测试：
  *          - 测试前进时电机方向是否正确
  *          - 测试前进死区是否过滤噪声
  *          - 测试安全模式是否正确清除状态并发送零命令
  *          - 测试安全模式后是否能恢复正常控制
  *          - 测试陀螺仪模式下平移是否跟随云台朝向
  *          - 测试陀螺仪模式下旋转是否朝向正角度方向
  *          - 测试 GYRO 不受机械模式 ALIGN_BACK 锁轴影响
  * @note   该测试使用桩函数（stub）模拟硬件操作，可在PC上独立运行
  ******************************************************************************
  */

/* 包含头文件 */
#include "chassis.h"        /* 底盘控制头文件 */
#include "chassis_motor.h"  /* 底盘电机控制头文件 */

#include <math.h>     /* 数学函数 */
#include <stdio.h>    /* 标准输入输出 */
#include <stdlib.h>   /* 标准库 */
#include <string.h>   /* 字符串操作（memset） */

/**
  * @brief 浮点数比较精度阈值
  */
#define EPSILON 0.001f

/**
  * @brief 满杆/满速比较阈值：cos(π)≠-1 的浮点尾巴，1 RPM 足够
  */
#define RPM_EPSILON 1.0f

/**
  * 全局变量 - 模拟底盘电机和控制结构体
  * 这些变量在测试中被初始化和验证
  */
Motor_RM_t Chassis_Motor[4];           /* 底盘四个电机实例 */
Motor_RM_Group_t Chassis_Motor_Group;  /* 电机组（用于批量控制） */

/**
  * 静态测试辅助变量 - 用于记录函数调用次数和状态
  */
static Motor_RM_Ctrl_Info_t motor_ctrl[4];   /* 电机控制信息 */
static Motor_RM_Tx_Info_t motor_tx[4];       /* 电机发送信息 */
static pid_ctrl_t speed_pid[4];              /* 速度PID控制器 */
static unsigned int speed_calls[4];          /* 速度设置函数调用次数 */
static unsigned int pid_clear_calls;         /* PID清除函数调用次数 */
static unsigned int group_send_calls;        /* 组发送函数调用次数 */

/* This fixture checks motion mapping. The real limiter/adapter is exercised
 * independently by test_power_integration.py; do not limit this fake PID. */
void Chassis_Power_Apply(void) {}

/**
  * @brief  测试失败处理函数
  * @param  message: 错误信息
  * @details 输出错误信息并退出程序
  * @retval None
  */
static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

/**
  * @brief  浮点数近似比较函数
  * @param  actual: 实际值
  * @param  expected: 期望值
  * @param  message: 错误信息
  * @details 如果实际值与期望值的差值超过阈值，则报告错误并退出
  * @retval None
  */
static void expect_near(float actual, float expected, const char *message)
{
    if (fabsf(actual - expected) >= EPSILON) {
        fprintf(stderr, "%s: expected %.3f, got %.3f\n",
                message, expected, actual);
        exit(EXIT_FAILURE);
    }
}

/**
  * @brief  满速近似比较（允许 cos(π) 浮点尾巴，误差 < 1 RPM）
  */
static void expect_near_rpm(float actual, float expected, const char *message)
{
    if (fabsf(actual - expected) >= RPM_EPSILON) {
        fprintf(stderr, "%s: expected %.3f, got %.3f\n",
                message, expected, actual);
        exit(EXIT_FAILURE);
    }
}

/**
  * @brief  PID控制器清除函数（桩函数）
  * @param  pid: PID控制器指针
  * @details 清除PID积分项并记录调用次数
  * @note   该函数是桩函数，用于替代真实的pid_clear函数
  * @retval None
  */
void pid_clear(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;    /* 清除积分项 */
    pid_clear_calls++;       /* 递增调用计数 */
}

/**
  * @brief  电机速度设置桩函数
  * @param  motor: 电机指针
  * @details 该桩函数模拟真实的电机速度设置操作：
  *          1. 记录调用次数
  *          2. 将PID目标值赋给电机转矩
  * @note   该函数用于验证速度PID是否被正确调用
  * @retval None
  */
static void fake_set_speed(Motor_RM_t *motor)
{
    /* 计算电机在数组中的索引 */
    const unsigned int index = (unsigned int)(motor - Chassis_Motor);
    speed_calls[index]++;   /* 记录该电机的速度设置调用次数 */

    /* 模拟：将PID目标值作为电机转矩输出 */
    motor->tx_info->torque = motor->ctrl->speed_ctrl->target;
}

/**
  * @brief  电机组发送桩函数
  * @param  group: 电机组指针
  * @details 记录组发送函数的调用次数
  * @note   该函数用于验证控制更新是否正确触发了电机通信
  * @retval None
  */
static void fake_group_send(Motor_RM_Group_t *group)
{
    (void)group;            /* 忽略未使用的参数 */
    group_send_calls++;     /* 递增调用计数 */
}

/**
  * @brief  测试夹具重置函数
  * @details 在每个测试用例执行前重置所有测试状态：
  *          1. 清零所有电机实例
  *          2. 清零所有控制信息和发送信息
  *          3. 重置PID控制器
  *          4. 清零所有调用计数
  *          5. 重新建立结构体之间的关联
  *          6. 设置桩函数指针
  * @retval None
  */
static void reset_fixture(void)
{
    /* 清零所有数据结构 */
    memset(Chassis_Motor, 0, sizeof(Chassis_Motor));
    memset(&Chassis_Motor_Group, 0, sizeof(Chassis_Motor_Group));
    memset(motor_ctrl, 0, sizeof(motor_ctrl));
    memset(motor_tx, 0, sizeof(motor_tx));
    memset(speed_pid, 0, sizeof(speed_pid));
    memset(speed_calls, 0, sizeof(speed_calls));
    pid_clear_calls = 0;
    group_send_calls = 0;

    /* 建立结构体之间的关联 */
    for (unsigned int i = 0; i < 4; ++i) {
        motor_ctrl[i].speed_ctrl = &speed_pid[i];      /* 控制信息指向PID */
        Chassis_Motor[i].ctrl = &motor_ctrl[i];        /* 电机指向控制信息 */
        Chassis_Motor[i].tx_info = &motor_tx[i];       /* 电机指向发送信息 */
        Chassis_Motor[i].single_set_speed = fake_set_speed;  /* 设置速度桩函数 */
        Chassis_Motor_Group.motor[i] = &Chassis_Motor[i];    /* 电机组指向电机 */
    }
    Chassis_Motor_Group.group_set_torque = fake_group_send;   /* 设置发送桩函数 */
}

/**
  * @brief  测试1：前进时电机方向是否正确
  * @details 验证底盘前进时，四个电机的转速符号是否正确：
  *         - 满杆 660 过死区后 vx=1，麦轮解算到 CHASSIS_MAX_MOTOR_RPM
  *         - 左右轮经方向系数后符号相反
  * @note   麦轮底盘前进时，左右轮转向相反
  * @retval None
  */
static void test_forward_applies_motor_directions(void)
{
    const float rpm = CHASSIS_MAX_MOTOR_RPM;
    const float expected[4] = {rpm, -rpm, rpm, -rpm};
    reset_fixture();

    /* 调用底盘更新函数：前进（vx=660），头朝前（delta=0） */
    Chassis_Update(660, 0, 0, 0.0f, ALIGN_FRONT, CAR_MODE_MECH, false);

    /* 验证每个电机的PID目标值 */
    for (unsigned int i = 0; i < 4; ++i) {
        expect_near_rpm(speed_pid[i].target, expected[i], "forward target");
        if (speed_calls[i] != 1) fail("speed controller must run once");
    }
    if (group_send_calls != 1) fail("normal update must send exactly once");
}

/**
  * @brief  测试2：前进死区是否过滤噪声
  * @details 验证 1 个计数的摇杆噪声被 CHASSIS_FORWARD_DEADZONE 滤掉：
  *          现网死区 = 6，死区内输出必须为 0，不能误冲车。
  * @retval None
  */
static void test_forward_deadzone_filters_noise(void)
{
    reset_fixture();

    /* 1 计数前进：在死区内，目标必须为 0 */
    Chassis_Update(1, 0, 0, 0.0f, ALIGN_FRONT, CAR_MODE_MECH, false);
    if (fabsf(speed_pid[0].target) >= EPSILON ||
        fabsf(speed_pid[1].target) >= EPSILON ||
        fabsf(speed_pid[2].target) >= EPSILON ||
        fabsf(speed_pid[3].target) >= EPSILON) {
        fail("one-count noise must be removed by CHASSIS_FORWARD_DEADZONE");
    }

    /* 死区外一点点：必须立刻有非零输出，不能被整段闸死 */
    Chassis_Update(CHASSIS_FORWARD_DEADZONE + 1, 0, 0, 0.0f,
                   ALIGN_FRONT, CAR_MODE_MECH, false);
    if (fabsf(speed_pid[0].target) < EPSILON) {
        fail("just-above-deadzone input must produce nonzero output");
    }
}

/**
  * @brief  测试3：安全模式是否正确清除状态并发送零命令
  * @details 验证安全模式（failsafe）触发时：
  *          1. 所有PID状态被清除
  *          2. 电机发送信息被清除
  *          3. PID积分被清除
  *          4. 发送一个零转矩命令
  *          5. 不执行速度PID计算
  * @note   安全模式用于紧急停止，确保系统安全
  * @retval None
  */
static void test_failsafe_clears_state_and_sends_zero_once(void)
{
    reset_fixture();

    /* 预设非零状态，用于验证清除效果 */
    for (unsigned int i = 0; i < 4; ++i) {
        speed_pid[i].target = 1.0f;
        speed_pid[i].measure = 2.0f;
        speed_pid[i].err = 3.0f;
        speed_pid[i].last_err = 4.0f;
        speed_pid[i].pout = 5.0f;
        speed_pid[i].iout = 6.0f;
        speed_pid[i].dout = 7.0f;
        speed_pid[i].last_dout = 8.0f;
        speed_pid[i].out = 9.0f;
        speed_pid[i].integral = 10.0f;
        motor_tx[i].torque = 11.0f;
        motor_tx[i].torque_current = 12.0f;
        motor_tx[i].torque_current_raw = 13;
    }

    /* 触发安全模式 */
    Chassis_Update(660, 660, 660, 0.0f, ALIGN_FRONT, CAR_MODE_GYRO, true);

    /* 验证所有状态被清除 */
    for (unsigned int i = 0; i < 4; ++i) {
        const float *fields[] = {
            &speed_pid[i].target, &speed_pid[i].measure, &speed_pid[i].err,
            &speed_pid[i].last_err, &speed_pid[i].pout, &speed_pid[i].iout,
            &speed_pid[i].dout, &speed_pid[i].last_dout, &speed_pid[i].out,
            &speed_pid[i].integral, &motor_tx[i].torque,
            &motor_tx[i].torque_current,
        };
        for (unsigned int field = 0; field < sizeof(fields) / sizeof(fields[0]); ++field) {
            expect_near(*fields[field], 0.0f, "failsafe state");
        }
        if (motor_tx[i].torque_current_raw != 0) fail("raw current was not cleared");
        if (speed_calls[i] != 0) fail("failsafe must not run speed PID");
    }
    if (pid_clear_calls != 4) fail("failsafe must clear all four PID integrals");
    if (group_send_calls != 1) fail("failsafe must send exactly one zero frame");
}

/**
  * @brief  测试4：安全模式后是否能恢复正常控制
  * @details 验证从安全模式恢复后，底盘控制能正常工作。
  *          满杆外的 330 计数过死区后按 (330-6)/(660-6)*MAX_RPM 缩放。
  * @retval None
  */
static void test_control_recovers_after_failsafe(void)
{
    const float expected = (324.0f / 654.0f) * CHASSIS_MAX_MOTOR_RPM;
    reset_fixture();

    /* 先触发安全模式 */
    Chassis_Update(0, 0, 0, 0.0f, ALIGN_FRONT, CAR_MODE_GYRO, true);
    group_send_calls = 0;  /* 重置发送计数 */

    /* 恢复正常控制：forward=330，MECH，头朝前 */
    Chassis_Update(330, 0, 0, 0.0f, ALIGN_FRONT, CAR_MODE_MECH, false);

    /* 验证恢复后的控制输出（满杆比例缩放，不是旧的 3000） */
    expect_near_rpm(speed_pid[0].target, expected, "recovery target");
    if (speed_calls[0] != 1 || group_send_calls != 1) {
        fail("normal control did not resume after failsafe");
    }
}

/**
  * @brief  测试5：陀螺仪模式下平移是否跟随云台朝向
  * @details 验证在陀螺仪模式下，底盘平移方向会根据云台朝向自动调整。
  *          头朝 +90° 时前进 = 底盘向左；跟随 wz 被限到 1.0。
  * @retval None
  */
static void test_gyro_translation_follows_gimbal_heading(void)
{
    const float half_pi = 1.57079632679f;  /* 90度（弧度） */
    const float rpm = CHASSIS_MAX_MOTOR_RPM;
    reset_fixture();

    /* 云台朝向90度时前进（ALIGN_FRONT：跟随误差=90°） */
    Chassis_Update(660, 0, 0, half_pi, ALIGN_FRONT, CAR_MODE_GYRO, false);

    /* vx'=0, vy'=1, wz=1 → 麦轮解算后方向系数下 FL/FR 为负 */
    expect_near_rpm(speed_pid[0].target, -rpm, "gyro FL target");
    expect_near_rpm(speed_pid[1].target, -rpm, "gyro FR target");
    expect_near(speed_pid[2].target, 0.0f, "gyro BL target");
    expect_near(speed_pid[3].target, 0.0f, "gyro BR target");
}

/**
  * @brief  测试6：陀螺仪模式下旋转是否朝向正角度方向
  * @details 验证在陀螺仪模式下，底盘旋转方向与云台朝向角度一致。
  * @retval None
  */
static void test_gyro_follow_rotates_toward_positive_angle(void)
{
    const float half_pi = 1.57079632679f;  /* 90度（弧度） */
    reset_fixture();

    /* 云台朝向90度时旋转 */
    Chassis_Update(0, 0, 0, half_pi, ALIGN_FRONT, CAR_MODE_GYRO, false);

    /* 验证旋转方向 - 正角度应该导致负转速（顺时针） */
    if (speed_pid[0].target >= 0.0f || speed_pid[1].target >= 0.0f ||
        speed_pid[2].target >= 0.0f || speed_pid[3].target >= 0.0f) {
        fail("positive gimbal angle must command negative chassis rotation targets");
    }
}

static void test_follow_deadzone_gain_and_limit(void)
{
    expect_near(Chassis_Calc_Follow_Wz_Norm(0.020f), 0.0f, "follow deadzone");
    expect_near(Chassis_Calc_Follow_Wz_Norm(0.026f), 0.0f, "follow continuity");
    expect_near(Chassis_Calc_Follow_Wz_Norm(0.5235988f), 0.398079f, "follow 30 degrees");
    expect_near(Chassis_Calc_Follow_Wz_Norm(-0.5235988f), -0.398079f, "follow negative");
    expect_near(Chassis_Calc_Follow_Wz_Norm(3.14159f), 1.0f, "follow positive limit");
    expect_near(Chassis_Calc_Follow_Wz_Norm(-3.14159f), -1.0f, "follow negative limit");
}

/**
  * @brief  测试8：GYRO 忽略机械模式 ALIGN_BACK，始终让车头跟随
  * @details 头朝车尾（delta=π）时：
  *          - FRONT：追 Δθ→π，底盘掉头半圈（wz 饱和）
  *          - BACK：同样追真实夹角，不使用 180° 就近对齐
  * @retval None
  */
static void test_gyro_ignores_mechanical_rear_alignment(void)
{
    const float pi = 3.14159265359f;
    const float rpm = CHASSIS_MAX_MOTOR_RPM;
    reset_fixture();

    /* BACK：机械模式的车尾锁轴状态不能改变 GYRO 跟随。 */
    Chassis_Update(660, 0, 0, pi, ALIGN_BACK, CAR_MODE_GYRO, false);
    /* vx=-1, wz=1，经轮速限幅后 FL/BL 为负满速，FR/BR 为零。 */
    expect_near_rpm(speed_pid[0].target, -rpm, "back FL reverse");
    expect_near(speed_pid[1].target, 0.0f, "back FR follows front");
    expect_near_rpm(speed_pid[2].target, -rpm, "back BL reverse");
    expect_near(speed_pid[3].target, 0.0f, "back BR follows front");

    /* FRONT：同一夹角仍要追 180° → wz 饱和，平移投影仍为 vx'=-1 */
    reset_fixture();
    Chassis_Update(660, 0, 0, pi, ALIGN_FRONT, CAR_MODE_GYRO, false);
    /* vx'=-1, wz=1 → FL=-2, FR=0, BL=-2, BR=0；scale=MAX/2 */
    expect_near_rpm(speed_pid[0].target, -rpm, "front FL after chase");
    expect_near(speed_pid[1].target, 0.0f, "front FR after chase");
    expect_near_rpm(speed_pid[2].target, -rpm, "front BL after chase");
    expect_near(speed_pid[3].target, 0.0f, "front BR after chase");
}

/**
  * @brief  主函数 - 测试入口
  * @details 依次执行所有测试用例
  * @retval int 返回EXIT_SUCCESS表示所有测试通过
  */
int main(void)
{
    test_forward_applies_motor_directions();       /* 测试1 */
    test_forward_deadzone_filters_noise();         /* 测试2 */
    test_failsafe_clears_state_and_sends_zero_once();  /* 测试3 */
    test_control_recovers_after_failsafe();        /* 测试4 */
    test_gyro_translation_follows_gimbal_heading(); /* 测试5 */
    test_gyro_follow_rotates_toward_positive_angle(); /* 测试6 */
    test_follow_deadzone_gain_and_limit();          /* 测试7 */
    test_gyro_ignores_mechanical_rear_alignment(); /* 测试8 */
    puts("8 chassis control tests passed");
    return EXIT_SUCCESS;
}
