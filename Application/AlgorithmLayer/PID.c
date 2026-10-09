/* 包含头文件 ------------------------------------------------------------------*/
#include "pid.h"      /* PID控制器头文件，包含结构体定义 */
#include "rp_math.h"  /* 数学库，提供constrain限幅宏 */


void single_pid_ctrl(pid_ctrl_t *pid)
{
    float dout = pid->kd * (pid->err - pid->last_err);
    dout = pid->a * dout + (1.0f - pid->a) * pid->last_dout;
    single_pid_ctrl_with_dout(pid, dout);
}

/* Accept an already filtered D term, retaining the shared P/I and limits. */
void single_pid_ctrl_with_dout(pid_ctrl_t *pid, float filtered_dout)
{
    /* 注意：误差err由外部计算后设置，不在函数内计算 */
    /* pid->err = pid->target - pid->measure; */

    pid->integral += pid->err;

    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);

    pid->pout = pid->kp * pid->err;

    pid->iout = pid->ki * pid->integral;

    pid->dout = filtered_dout;
    pid->last_dout = pid->dout;  /* 关键：务必更新并保存本次滤波后的微分值！ */

    /* ===== 累加PID输出 ===== */
    pid->out = pid->pout + pid->iout + pid->dout;


    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);

    /* 记录本次误差值，供下次微分计算使用 */
    pid->last_err = pid->err;
}


void pid_err_cal(pid_ctrl_t *pid)
{
    pid->err = pid->target - pid->measure;
}

/*清零PID控制器的积分*/
void pid_clear(pid_ctrl_t *pid) {
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
    pid->last_dout = 0.0f;
}
