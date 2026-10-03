#include "pid.h"
#include "rp_math.h"  /* 引入限幅函数 constrain */
#include <string.h>
/*云台角度环防大回旋（最短路径算法）：
云台的角度误差不能直接写 target - measure会发生大回旋，
必须经过 Calc_Shortest_Angle_Err() 算出最短路径误差，所以误差被强行挪到了外面算

微分先行 / 低通滤波 D 项 / 陀螺仪角速度直接作为 D：
在做云台和底盘控制时，很多地方的 D 项不是直接差分算出来的，
而是直接拿陀螺仪的角速度（原生高频微分），或者经过了一阶低通滤波。
老队员为了“复用”同一个 PID 结构体，
就把算 D 项的逻辑剥离到了另一个独立的函数（比如 pid_err_cal()）或者应用层里去算。
*/

//P + I 累加与限幅输出器


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



/*    调用它的人必须提前把 pid->err 算好塞进结构体，
       本函数里面完全没有计算 D 项
*/

void single_pid_ctrl(pid_ctrl_t *pid)
{

	/* 累加误差到积分器 */
	// pid->err = pid->target - pid->measure;  /* 误差已在外部计算完毕 */
	pid->integral += pid->err;

	pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);


	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;

	/* 保存上一次的D项输出（D项由外部微分计算后赋值给 pid->dout） */
	pid->last_dout = pid->dout;

	pid->out = pid->pout + pid->iout + pid->dout;


	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);

	/* 保存本次误差，供下次调用时作为"上一次误差"使用 */
	pid->last_err = pid->err;
}
