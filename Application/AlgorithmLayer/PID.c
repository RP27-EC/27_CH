#include "pid.h"
#include "rp_math.h"

void single_pid_ctrl(pid_ctrl_t *pid)
{
    // 保存误差值(需要在外面自行计算误差)
//		pid->err = pid->target - pid->measure;
    // 积分
    pid->integral += pid->err;
    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
    // p i d 输出项计算
    pid->pout = pid->kp * pid->err;
    pid->iout = pid->ki * pid->integral;
    pid->dout = pid->kd * (pid->err - pid->last_err);
		// 微分低通滤波
		pid->dout = pid->a * pid->dout + (1.f - pid->a) * pid->last_dout;
    // 累加pid输出值
    pid->out = pid->pout + pid->iout + pid->dout;
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
    // 记录上次误差值
    pid->last_err = pid->err;
}

void pid_err_cal(pid_ctrl_t *pid)
{
	pid->err = pid->target - pid->measure;
}

/*离线状态下对积分清零*/
void pid_clear(pid_ctrl_t *pid)
{
	pid->integral = 0;
}

//pid外部调试窗口
void pid_watch(pid_ctrl_t *pid1, pid_ctrl_t *pid2, pid_watch_t * watch)
{
	watch->measure1 = pid1->measure;
	watch->target1 = pid1->target;
	watch->measure2 = pid2->measure;
	watch->target2 = pid2->target;
	watch->out1 = pid1->out;
	watch->out2 = pid2->out;
	watch->err1 = pid1->err;
	watch->err2 = pid2->err;
}
