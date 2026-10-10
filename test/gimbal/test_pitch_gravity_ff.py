"""Execute the firmware pitch gravity feedforward with host stubs."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def firmware_harness():
    header = (ROOT / 'Application/ModuleLayer/gimbal.h').read_text(encoding='utf-8-sig')
    source = (ROOT / 'Application/ModuleLayer/gimbal.c').read_text(encoding='utf-8-sig')
    end = header.index('} Gimbal_Pitch_t;') + len('} Gimbal_Pitch_t;')
    pitch_type = header[header.rfind('typedef struct {', 0, end):end]
    deg = next(line for line in source.splitlines()
               if re.match(r'#define\s+DEG_TO_RAD', line))

    def func(signature):
        start = source.index(signature + '\n{')
        return source[start:source.index('\n}', start) + 2]

    funcs = '\n'.join(func(sig) for sig in (
        'static void Gimbal_Pitch_Clear_Pids(void)',
        'void Gimbal_Safe_Stop(void)',
        'static float Pitch_Gravity_Compensation(float pitch_rad)',
        'static void Gimbal_Pitch_Pid_Cal(void)',
    ))
    return r'''
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
typedef struct {
    float target, measure, err, last_err, kp, ki, kd;
    float pout, iout, dout, out, last_dout, integral, integral_max, out_max;
} pid_ctrl_t;
static void pid_clear(pid_ctrl_t *p){ *p=(pid_ctrl_t){0}; }
static void pid_err_cal(pid_ctrl_t *p){ p->err=p->target-p->measure; }
static void single_pid_ctrl(pid_ctrl_t *p){ p->out=p->kp*p->err; }  /* 零增益时输出 0，便于孤立前馈项 */
''' + deg + '\n' + pitch_type + r'''
typedef struct { Gimbal_Pitch_t pitch; } Gimbal_t;
struct { bool is_online; bool pitch_enable; } board_rx_info;
typedef struct { float torque; } MotorTxInfo_t;
typedef struct {
    MotorTxInfo_t *tx_info;
    void (*single_set_torque)(void *);
    void (*single_sleep)(void *);
} Motor_t;
static MotorTxInfo_t tx_info;
static void nop(void *m){ (void)m; }
static Motor_t Pitch_Motor;
static Gimbal_t gimbal;
''' + funcs + r'''
#define g gimbal

static void setup(void){
    gimbal=(Gimbal_t){0};
    Pitch_Motor.tx_info=&tx_info;
    Pitch_Motor.single_set_torque=nop;
    Pitch_Motor.single_sleep=nop;
    board_rx_info.is_online=true;board_rx_info.pitch_enable=true;
    gimbal.pitch.gravity_max_torque=2.0f;
    gimbal.pitch.gravity_phase_deg=0.0f;
    gimbal.pitch.max_torque_limit=5.5f;   /* 不设的话输出限幅会把前馈项截成 0 */
}
static void near(float a,float b){if(fabsf(a-b)>=1e-6f){fprintf(stderr,"near fail: %g vs %g\n",a,b);exit(1);}}
'''


class PitchGravityFfTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.harness = firmware_harness()

    def run_case(self, body):
        with tempfile.TemporaryDirectory() as directory:
            c = pathlib.Path(directory) / 'pitch.c'
            exe = pathlib.Path(directory) / 'pitch.exe'
            c.write_text(self.harness + '\nint main(void){setup();\n' + body + '\nreturn 0;}\n', encoding='utf-8')
            subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Wno-unused-function', str(c), '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_ff_is_one_cos_model(self):
        self.run_case(r'''
    near(Pitch_Gravity_Compensation(0.0f),2.0f);
    near(Pitch_Gravity_Compensation(0.3f),2.0f*cosf(0.3f));
    near(Pitch_Gravity_Compensation(-0.3f),2.0f*cosf(-0.3f));
    /* 相位旋钮 */
    g.pitch.gravity_phase_deg=90.0f;
    near(Pitch_Gravity_Compensation(0.0f),0.0f);
    /* 符号旋钮：幅值取负即反向 */
    g.pitch.gravity_phase_deg=0.0f;g.pitch.gravity_max_torque=-2.0f;
    near(Pitch_Gravity_Compensation(0.0f),-2.0f);
''')

    def test_ff_does_not_depend_on_mode(self):
        self.run_case(r'''
    g.pitch.use_imu=true; float a=Pitch_Gravity_Compensation(0.4f);
    g.pitch.use_imu=false;float b=Pitch_Gravity_Compensation(0.4f);
    near(a,b);near(a,2.0f*cosf(0.4f));
''')

    def test_loop_feeds_encoder_angle_to_ff(self):
        """闭环反馈随 use_imu 切换，前馈永远吃编码器机械角。"""
        self.run_case(r'''
    g.pitch.use_imu=true;
    g.pitch.feedback_pitch=0.3f;   /* 编码器 */
    g.pitch.imu_pitch=0.6f;        /* IMU：故意不同 */
    g.pitch.target_pitch=0.6f;     /* 对准 IMU 反馈，闭环误差为 0，输出即纯前馈 */
    Gimbal_Pitch_Pid_Cal();
    near(g.pitch.give_torque,2.0f*cosf(0.3f));                    /* 吃编码器 */
    CHECK(fabsf(g.pitch.give_torque-2.0f*cosf(0.6f))>1e-3f);      /* 没吃 IMU */
    near(tx_info.torque,g.pitch.give_torque);
    /* 对调两个反馈再验一次 */
    g.pitch.feedback_pitch=0.6f;g.pitch.imu_pitch=0.3f;
    Gimbal_Pitch_Pid_Cal();
    near(g.pitch.give_torque,2.0f*cosf(0.6f));
    /* MECH 模式同一条曲线同一个输入 */
    g.pitch.use_imu=false;g.pitch.target_pitch=0.6f;
    Gimbal_Pitch_Pid_Cal();
    near(g.pitch.give_torque,2.0f*cosf(0.6f));
''')

    def test_loop_uses_imu_angle_outer_and_imu_rate_inner(self):
        """GYRO uses IMU feedback for both angle and rate."""
        self.run_case(r'''
    g.pitch.use_imu=true;
    g.pitch.gravity_max_torque=0.0f;      /* 关前馈，孤立 PID 项 */
    g.pitch.imu_angle_pid.kp=1.0f;
    g.pitch.imu_speed_pid.kp=1.0f;
    g.pitch.target_pitch=0.6f;g.pitch.imu_pitch=0.3f;g.pitch.feedback_pitch=0.0f;
    g.pitch.feedback_speed=0.0f;g.pitch.imu_speed=0.5f;  /* 陀螺速率故意不同 */
    /* Outer: 0.6-0.3=0.3; inner: 0.3-0.5=-0.2; torque=+0.2. */
    Gimbal_Pitch_Pid_Cal();
    near(g.pitch.give_torque,0.2f);
''')


if __name__ == '__main__':
    unittest.main()
