"""Run the firmware Yaw controller and PID on the host."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def function(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start) + 2]


def harness():
    source = (ROOT / 'Application/ModuleLayer/gimbal.c').read_text(encoding='utf-8-sig')
    header = (ROOT / 'Application/ModuleLayer/gimbal.h').read_text(encoding='utf-8-sig')
    pid_header = (ROOT / 'Application/AlgorithmLayer/PID.h').read_text(encoding='utf-8-sig')
    pid_source = (ROOT / 'Application/AlgorithmLayer/PID.c').read_text(encoding='utf-8-sig')
    functions = '\n'.join(function(source, signature) for signature in (
        'static float Angle_Normalize_PI(', 'static float Calc_Shortest_Angle_Err(',
        'static pid_ctrl_t *YawAnglePid(', 'static pid_ctrl_t *YawSpeedPid(',
        'static void ClearYawPids(', 'void Gimbal_Init(', 'static void Gimbal_Yaw_Pid_Cal('))
    return r'''
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159265358979323846f
#define constrain(x,lo,hi) ((x)>(hi)?(hi):((x)<(lo)?(lo):(x)))
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
typedef enum {ALIGN_FRONT, ALIGN_BACK} AlignDir_e;
typedef int CarMode_e;
''' + pid_header + re.sub(r'^#include[^\n]*', '', header, flags=re.M) + r'''
struct {bool is_online;} board_rx_info;
enum {DEV_OFFLINE, DEV_ONLINE};
enum {IMU_NONE_ERR, IMU_DATA_ERR, IMU_DATA_CALI};
typedef struct {struct {float ave_rate_yaw;} base_info;} HostImuInfo;
static HostImuInfo bottom_imu_info;
struct {
    HostImuInfo *info;
    struct {int dev_state, err_code, cali_end;} work_state;
} imu_sensor;
struct {struct {int16_t motor_out;} base_info;} kt_motor[1];
static int16_t sent_current;
static void KT_Yaw_SendIq(int16_t current){sent_current=current;}
''' + re.sub(r'^#include[^\n]*', '', pid_source, flags=re.M) + functions + r'''
static Gimbal_t g;
static void near(float a,float b){CHECK(fabsf(a-b)<1e-5f);}
static void setup(void){
    g=(Gimbal_t){0};Gimbal_Init(&g);
    g.mode=GIMBAL_MODE_GYRO;g.yaw.pid_mode=GIMBAL_PID_GYRO;
    board_rx_info.is_online=true;
    bottom_imu_info=(HostImuInfo){0};imu_sensor.info=&bottom_imu_info;
    imu_sensor.work_state.dev_state=DEV_ONLINE;
    imu_sensor.work_state.err_code=IMU_NONE_ERR;
    imu_sensor.work_state.cali_end=1;
    g.yaw.gyro_speed_pid.kd=20.0f;g.yaw.gyro_speed_pid.a=.2f;
    Gimbal_Yaw_Pid_Cal(&g);
}
'''


class YawDerivativeTest(unittest.TestCase):
    def run_case(self, body):
        with tempfile.TemporaryDirectory() as directory:
            src = pathlib.Path(directory) / 'yaw.c'
            exe = pathlib.Path(directory) / 'yaw.exe'
            src.write_text(harness() + '\nint main(void){setup();\n' + body + '\n}\n', encoding='utf-8')
            subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', str(src), '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_target_step_does_not_kick_speed_derivative(self):
        self.run_case(r'''
    g.yaw.target_imu_angle=1.0f;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_angle_pid.dout,0);near(g.yaw.gyro_speed_pid.dout,0);
    CHECK(g.yaw.gyro_angle_pid.pout>0 && g.yaw.gyro_speed_pid.pout>0);
''')

    def test_derivative_opposes_motion_and_lowpass_reduces_noise(self):
        self.run_case(r'''
    g.yaw.imu_angle=.004f;g.yaw.imu_speed=2.0f;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_speed_pid.dout,-8.0f);
    for(int i=0;i<200;i++){
        g.yaw.imu_speed=(i%2)?2.0f:-2.0f;Gimbal_Yaw_Pid_Cal(&g);
    }
    CHECK(fabsf(g.yaw.gyro_speed_pid.dout)<12.0f);
''')

    def test_angle_wrap_does_not_kick_speed_derivative(self):
        self.run_case(r'''
    g.yaw.imu_angle=PI-.001f;g.yaw.imu_speed=1.0f;Gimbal_Yaw_Pid_Cal(&g);
    g.yaw.imu_angle=-PI+.001f;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_speed_pid.dout,-3.2f);
''')

    def test_sleep_and_offline_clear_history(self):
        self.run_case(r'''
    g.yaw.imu_speed=2;Gimbal_Yaw_Pid_Cal(&g);
    g.mode=GIMBAL_MODE_SLEEP;Gimbal_Yaw_Pid_Cal(&g);
    CHECK(sent_current==0);near(g.yaw.gyro_angle_pid.last_dout,0);
    g.mode=GIMBAL_MODE_GYRO;g.yaw.imu_speed=3;g.yaw.imu_angle=1;
    Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_angle_pid.dout,0);near(g.yaw.gyro_speed_pid.dout,0);
    board_rx_info.is_online=false;Gimbal_Yaw_Pid_Cal(&g);CHECK(sent_current==0);
    board_rx_info.is_online=true;g.yaw.imu_speed=-3;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_angle_pid.dout,0);near(g.yaw.gyro_speed_pid.dout,0);
''')

    def test_feedback_source_switch_primes_derivative(self):
        self.run_case(r'''
    g.yaw.imu_speed=2;Gimbal_Yaw_Pid_Cal(&g);
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    g.yaw.mec_speed_pid.kd=20;g.yaw.mec_speed_pid.a=.2f;
    g.yaw.feedback_mec_angle=1;g.yaw.feedback_mec_speed=-3;
    Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_angle_pid.dout,0);near(g.yaw.mec_speed_pid.dout,0);
''')

    def test_zero_kd_preserves_proportional_and_integral_output(self):
        self.run_case(r'''
    g.yaw.gyro_angle_pid.kp=25;g.yaw.gyro_speed_pid.kp=560;
    g.yaw.gyro_angle_pid.kd=g.yaw.gyro_speed_pid.kd=0;
    g.yaw.gyro_speed_pid.ki=.1f;g.yaw.target_imu_angle=.01f;
    g.yaw.imu_speed=.1f;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_angle_pid.out,.25f);
    near(g.yaw.gyro_speed_pid.out,560*.15f+.1f*.15f);
''')

    def test_watch_zero_kd_disables_derivative_immediately(self):
        self.run_case(r'''
    g.yaw.imu_speed=2;Gimbal_Yaw_Pid_Cal(&g);
    CHECK(g.yaw.gyro_speed_pid.dout!=0);
    g.yaw.gyro_speed_pid.kd=0;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_speed_pid.dout,0);near(g.yaw.gyro_speed_pid.last_dout,0);
''')

    def test_mechanical_inner_loop_filters_feedback_too(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    g.yaw.mec_speed_pid.kd=20;
    Gimbal_Yaw_Pid_Cal(&g);
    g.yaw.imu_speed=2;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_speed_pid.dout,-8);
    g.yaw.mec_speed_pid.a=1;g.yaw.imu_speed=3;
    Gimbal_Yaw_Pid_Cal(&g);near(g.yaw.mec_speed_pid.dout,-20);
''')

    def test_shared_pid_keeps_legacy_results_and_limits(self):
        self.run_case(r'''
    pid_ctrl_t p={0}, ref={0};
    p.kp=ref.kp=3;p.ki=ref.ki=.5f;p.kd=ref.kd=2;
    p.a=ref.a=.2f;p.out_max=ref.out_max=4;
    p.integral_max=ref.integral_max=2;
    const float errors[]={1,2,4,-3,-4,0,.2f};
    for(unsigned i=0;i<sizeof(errors)/sizeof(errors[0]);i++){
        p.err=ref.err=errors[i];
        ref.integral+=ref.err;
        ref.integral=constrain(ref.integral,-ref.integral_max,ref.integral_max);
        ref.pout=ref.kp*ref.err;ref.iout=ref.ki*ref.integral;
        ref.dout=ref.kd*(ref.err-ref.last_err);
        ref.dout=ref.a*ref.dout+(1-ref.a)*ref.last_dout;
        ref.last_dout=ref.dout;
        ref.out=ref.pout+ref.iout+ref.dout;
        ref.out=constrain(ref.out,-ref.out_max,ref.out_max);
        ref.last_err=ref.err;
        single_pid_ctrl(&p);
        near(p.integral,ref.integral);near(p.pout,ref.pout);
        near(p.iout,ref.iout);near(p.dout,ref.dout);near(p.out,ref.out);
        near(p.last_dout,ref.last_dout);near(p.last_err,ref.last_err);
    }
    pid_clear(&p);near(p.integral,0);near(p.dout,0);near(p.last_dout,0);
    near(p.err,0);near(p.last_err,0);near(p.pout,0);near(p.iout,0);near(p.out,0);
    near(p.kp,3);near(p.ki,.5f);near(p.kd,2);near(p.a,.2f);
''')


if __name__ == '__main__':
    unittest.main()
