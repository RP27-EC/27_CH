#ifndef __PID_H
#define __PID_H
//#include "pid_conf.h"
typedef struct pid_ctrl {
	float	target;
	float	measure;
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	last_dout;
	float 	out;
	float	integral;
	float 	integral_max;
	float 	out_max;
	float		a;
} pid_ctrl_t;

void single_pid_ctrl(pid_ctrl_t *pid);
void pid_err_cal(pid_ctrl_t *pid);
void pid_clear(pid_ctrl_t *pid);

#endif
