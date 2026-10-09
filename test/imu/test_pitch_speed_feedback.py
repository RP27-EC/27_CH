"""Run the firmware Pitch loop with distinct encoder and IMU feedback."""
import unittest

from test_ekf_init import ROOT, run_c


class PitchSpeedFeedbackTest(unittest.TestCase):
    def test_both_modes_use_imu_speed_with_separate_pids(self):
        source = (ROOT / 'Application/ModuleLayer/gimbal.c').read_text(encoding='utf-8-sig')
        start = source.index('static void Gimbal_Pitch_Pid_Cal(void)\n{')
        pitch_loop = source[start:source.index('\n}', start) + 2]
        run_c(r'''
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <assert.h>
#define DEG_TO_RAD 0.01745329251994329577f
typedef struct {float target,measure,out,kp,out_max;} pid_ctrl_t;
struct {struct {
    bool use_imu;
    float feedback_pitch,feedback_speed,imu_pitch,imu_speed,target_pitch;
    float max_speed_limit,max_torque_limit,give_torque;
    pid_ctrl_t mech_angle_pid,mech_speed_pid,imu_angle_pid,imu_speed_pid;
} pitch;} gimbal;
struct {bool is_online,pitch_enable;} board_rx_info={true,true};
struct tx_type {float torque;} motor_tx;
struct actual_motor {
    void (*single_set_torque)(struct actual_motor*);
    struct tx_type *tx_info;
} Pitch_Motor;
static int sends;
static void send(struct actual_motor *motor){sends++;}
static void Gimbal_Safe_Stop(void){assert(false);}
/* Isolate the feedback path from gravity compensation. */
static float Pitch_Gravity_Compensation(float angle){return 0;}
static void pid_err_cal(pid_ctrl_t *pid){}
static void single_pid_ctrl(pid_ctrl_t *pid){pid->out=pid->kp*(pid->target-pid->measure);}
static void near(float a,float b){assert(fabsf(a-b)<1e-6f);}
''' + pitch_loop + r'''
int main(void){
    Pitch_Motor.single_set_torque=send;Pitch_Motor.tx_info=&motor_tx;
    gimbal.pitch.max_speed_limit=6;gimbal.pitch.max_torque_limit=5.5f;
    gimbal.pitch.mech_angle_pid.kp=2;gimbal.pitch.mech_speed_pid.kp=3;
    gimbal.pitch.imu_angle_pid.kp=1;gimbal.pitch.imu_speed_pid.kp=1;
    gimbal.pitch.target_pitch=.6f;gimbal.pitch.feedback_pitch=.4f;
    gimbal.pitch.imu_pitch=.1f;gimbal.pitch.feedback_speed=-.7f;
    gimbal.pitch.imu_speed=.25f;
    Gimbal_Pitch_Pid_Cal();
    near(gimbal.pitch.mech_angle_pid.measure,.4f);
    near(gimbal.pitch.mech_speed_pid.target,.4f);
    near(gimbal.pitch.mech_speed_pid.measure,.25f);
    near(gimbal.pitch.give_torque,-.45f);
    near(gimbal.pitch.imu_speed_pid.out,0);
    gimbal.pitch.feedback_speed=4;
    Gimbal_Pitch_Pid_Cal();near(gimbal.pitch.give_torque,-.45f);
    gimbal.pitch.imu_speed=-.25f;
    Gimbal_Pitch_Pid_Cal();near(gimbal.pitch.give_torque,-1.95f);
    gimbal.pitch.use_imu=true;
    Gimbal_Pitch_Pid_Cal();
    near(gimbal.pitch.imu_angle_pid.measure,.1f);
    near(gimbal.pitch.imu_speed_pid.target,.5f);
    near(gimbal.pitch.imu_speed_pid.measure,-.25f);
    near(gimbal.pitch.give_torque,-.75f);
    near(motor_tx.torque,gimbal.pitch.give_torque);
    assert(sends==4);
}
''')


if __name__ == '__main__':
    unittest.main()
