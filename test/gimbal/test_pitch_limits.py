"""Execute the firmware Pitch target update with host input/feedback stubs."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def firmware_harness():
    header = (ROOT / 'Application/ModuleLayer/gimbal.h').read_text(encoding='utf-8-sig')
    source = (ROOT / 'Application/ModuleLayer/gimbal.c').read_text(encoding='utf-8-sig')
    end = header.index('} Gimbal_Pitch_Target_t;') + len('} Gimbal_Pitch_Target_t;')
    pitch_type = header[header.rfind('typedef struct {', 0, end):end]
    constants = '\n'.join(line for line in header.splitlines()
                          if re.match(r'#define\s+(DEG_TO_RAD|PITCH_|RC_PITCH_)', line))
    start = source.index('static void Gimbal_Pitch_Update(Gimbal_t *gimbal)\n{')
    update = source[start:source.index('\n}', start) + 2]
    return r'''
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
''' + constants + '\n' + pitch_type + r'''
enum {GIMBAL_MODE_SLEEP, GIMBAL_MODE_INIT, GIMBAL_MODE_MECH, GIMBAL_MODE_GYRO};
enum {INPUT_SRC_RC, INPUT_SRC_KEY};
typedef struct {int mode,last_mode; Gimbal_Pitch_Target_t pitch;} Gimbal_t;
struct {bool is_pitch_online,is_pitch_mech_online;float pitch_mech_angle;} board_rx_info;
struct {int input_src;} hero_car;
struct rc_info_t {float ch1;} rc_info;
struct {struct rc_info_t *info;} rc_sensor;
static float key_delta;
static int key_calls;
static void Gimbal_Key_Pitch_Update(float *target){key_calls++;*target+=key_delta;key_delta=0;}
''' + update + r'''
static Gimbal_t g;
static void setup(void){
    g=(Gimbal_t){0};g.mode=g.last_mode=GIMBAL_MODE_GYRO;
    g.pitch.min_limit=PITCH_MIN_MEC_ANGLE;g.pitch.max_limit=PITCH_MAX_MEC_ANGLE;
    g.pitch.imu_angle=g.pitch.target_imu_angle=.15f;
    board_rx_info.pitch_mech_angle=0;
    board_rx_info.is_pitch_online=board_rx_info.is_pitch_mech_online=true;
    rc_sensor.info=&rc_info;rc_info.ch1=0;hero_car.input_src=INPUT_SRC_RC;
}
static void step(float input){rc_info.ch1=input;Gimbal_Pitch_Update(&g);g.last_mode=g.mode;}
static void saturate(float input){for(int i=0;i<400;i++)step(input);}
static void near(float a,float b){CHECK(fabsf(a-b)<1e-6f);}
'''


class PitchLimitTest(unittest.TestCase):
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

    def test_full_stick_is_flat_at_limits_despite_feedback_jitter(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.run_case(f'float input={direction}*660.0f;\n' + r'''
    saturate(input);
    float held=g.pitch.target_imu_angle;
    near(held, input>0 ? PITCH_MAX_IMU_ANGLE : PITCH_MIN_IMU_ANGLE);
    /* 采样时差噪声/反馈抖动：边界是常数，目标一个数位都不动。 */
    for(int i=0;i<1000;i++){
        float jitter=(i%2 ? .001f : -.001f);
        g.pitch.imu_angle=held+jitter;
        board_rx_info.pitch_mech_angle=held-.15f-jitter;
        step(input);CHECK(g.pitch.target_imu_angle==held);
    }
    /* 车身倾斜也一样：静态限位不跟姿态，目标仍是水平直线。 */
    g.pitch.imu_angle+=0.04f*(input>0 ? -1 : 1);
    for(int i=0;i<100;i++){step(input);CHECK(g.pitch.target_imu_angle==held);}
''')

    def test_reverse_leaves_limit_on_first_tick(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.run_case(f'float input={direction}*660.0f;\n' + r'''
    saturate(input);float held=g.pitch.target_imu_angle;
    step(-input);
    near(g.pitch.target_imu_angle, held-input*RC_PITCH_MECH_SENSITIVITY);
    float retreated=g.pitch.target_imu_angle;
    step(0);CHECK(g.pitch.target_imu_angle==retreated);
''')

    def test_interior_target_stays_in_world_frame(self):
        self.run_case(r'''
    g.pitch.target_imu_angle=.4f;g.pitch.imu_angle=.4f;
    step(0);near(g.pitch.target_imu_angle,.4f);
    /* 行程内目标不跟随反馈平移，只在越界时截断。 */
    g.pitch.imu_angle+=.05f;step(0);near(g.pitch.target_imu_angle,.4f);
    g.pitch.imu_angle-=.3f;step(0);near(g.pitch.target_imu_angle,.4f);
''')

    def test_mech_mode_clamps_to_encoder_range(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;board_rx_info.pitch_mech_angle=.2f;step(0);
    near(g.pitch.target_mec_angle,.2f);
    saturate(660);near(g.pitch.target_mec_angle,g.pitch.max_limit);
    saturate(-660);near(g.pitch.target_mec_angle,g.pitch.min_limit);
''')

    def test_feedback_loss_rebases_before_accepting_input(self):
        self.run_case(r'''
    saturate(660);board_rx_info.is_pitch_mech_online=false;
    g.pitch.imu_angle=.3f;step(660);
    CHECK(g.pitch.target_rebase_pending);near(g.pitch.target_imu_angle,.3f);
    board_rx_info.is_pitch_mech_online=true;g.pitch.imu_angle=.5f;
    step(0);CHECK(!g.pitch.target_rebase_pending);near(g.pitch.target_imu_angle,.5f);
    saturate(660);near(g.pitch.target_imu_angle,PITCH_MAX_IMU_ANGLE);
''')

    def test_sleep_and_mode_changes_follow_current_feedback(self):
        self.run_case(r'''
    saturate(660);g.mode=GIMBAL_MODE_SLEEP;step(660);
    near(g.pitch.target_imu_angle,g.pitch.imu_angle);
    g.mode=GIMBAL_MODE_GYRO;g.pitch.imu_angle=.5f;
    g.pitch.target_imu_angle=.5f;saturate(660);
    near(g.pitch.target_imu_angle,PITCH_MAX_IMU_ANGLE);
    g.mode=GIMBAL_MODE_MECH;board_rx_info.pitch_mech_angle=.2f;step(0);
    near(g.pitch.target_mec_angle,.2f);
    saturate(-660);near(g.pitch.target_mec_angle,g.pitch.min_limit);
    g.mode=GIMBAL_MODE_GYRO;board_rx_info.pitch_mech_angle=0;
    g.pitch.imu_angle=0;g.pitch.target_imu_angle=0;saturate(660);
    near(g.pitch.target_imu_angle,PITCH_MAX_IMU_ANGLE);
''')

    def test_keyboard_uses_same_limits(self):
        self.run_case(r'''
    hero_car.input_src=INPUT_SRC_KEY;
    key_delta=2;step(0);near(g.pitch.target_imu_angle,PITCH_MAX_IMU_ANGLE);
    for(int i=0;i<100;i++){key_delta=.1f;step(0);CHECK(g.pitch.target_imu_angle==PITCH_MAX_IMU_ANGLE);}
    key_delta=-.01f;step(0);near(g.pitch.target_imu_angle,PITCH_MAX_IMU_ANGLE-.01f);
    step(0);near(g.pitch.target_imu_angle,PITCH_MAX_IMU_ANGLE-.01f);CHECK(key_calls==103);
''')


if __name__ == '__main__':
    unittest.main()
