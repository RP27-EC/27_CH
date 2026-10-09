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

    def test_full_stick_holds_both_limits_despite_feedback_jitter(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.run_case(f'float input={direction}*660.0f;\n' + r'''
    saturate(input);float held=g.pitch.target_imu_angle;
    for(int i=0;i<1000;i++){
        float jitter=(i%2 ? .001f : -.001f);
        /* Opposite sensor errors also model non-simultaneous CAN samples. */
        g.pitch.imu_angle=held+jitter;
        board_rx_info.pitch_mech_angle=held-.15f-jitter;
        step(input);CHECK(g.pitch.target_imu_angle==held);
    }
    step(-input);CHECK((g.pitch.target_imu_angle-held)*input<0);
    float retreated=g.pitch.target_imu_angle;
    step(0);CHECK(g.pitch.target_imu_angle==retreated);
''')

    def test_small_reverse_then_outward_cannot_ratchet_past_limit(self):
        self.run_case(r'''
    saturate(660);float held=g.pitch.target_imu_angle;
    for(int i=0;i<1000;i++){
        step(-40);CHECK(g.pitch.target_imu_angle<held);
        g.pitch.imu_angle=.15f+(i%2 ? .001f : -.001f);
        step(660);CHECK(g.pitch.target_imu_angle==held);
    }
''')

    def test_held_target_survives_body_tilt_until_input_reverses(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.run_case(f'float input={direction}*660.0f;\n' + r'''
    saturate(input);float held=g.pitch.target_imu_angle;
    /* 车身倾斜超过锁存阈值：边界整体平移，贴界目标仍一个数位不动。 */
    g.pitch.imu_angle+=0.04f*(input>0 ? -1 : 1);
    for(int i=0;i<100;i++){step(input);CHECK(g.pitch.target_imu_angle==held);}
    /* 反向输入立即释放，目标离开旧边界。 */
    step(-input);CHECK(g.pitch.target_imu_angle!=held);
    /* 再回推时贴上平移后的新边界，且新边界处再次给死。 */
    for(int i=0;i<50;i++)step(input);
    float reheld=g.pitch.target_imu_angle;
    for(int i=0;i<100;i++){step(input);CHECK(g.pitch.target_imu_angle==reheld);}
''')

    def test_bound_has_margin_for_untracked_small_frame_changes(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.run_case(f'float input={direction}*660.0f;\n' + r'''
    saturate(input);
    float new_offset=.15f+(input>0 ? -.003f : .003f);
    float implied_mech=g.pitch.target_imu_angle-new_offset;
    CHECK(implied_mech>=g.pitch.min_limit && implied_mech<=g.pitch.max_limit);
''')

    def test_interior_target_stays_in_world_frame(self):
        self.run_case(r'''
    g.pitch.target_imu_angle=.2f;g.pitch.imu_angle=.2f;
    step(0);near(g.pitch.target_imu_angle,.2f);
    /* 行程内目标不跟随反馈平移，只在越界时截断。 */
    g.pitch.imu_angle+=.05f;step(0);near(g.pitch.target_imu_angle,.2f);
''')

    def test_release_reclamps_frozen_target_into_reachable_range(self):
        self.run_case(r'''
    saturate(660);float held=g.pitch.target_imu_angle;
    /* 车身倾到旧目标不可达：外打期间目标保持不动。 */
    g.pitch.imu_angle-=.04f;
    for(int i=0;i<10;i++){step(660);CHECK(g.pitch.target_imu_angle==held);}
    /* 反向输入释放的当拍，目标被截回机械行程对应的世界系区间。 */
    step(-660);
    CHECK(g.pitch.target_imu_angle<=g.pitch.imu_limit_offset+g.pitch.max_limit-PITCH_LIMIT_MARGIN_RAD+1e-6f);
''')

    def test_feedback_loss_rebases_before_accepting_input(self):
        self.run_case(r'''
    saturate(660);board_rx_info.is_pitch_mech_online=false;
    g.pitch.imu_angle=.3f;step(660);
    CHECK(g.pitch.target_rebase_pending);near(g.pitch.target_imu_angle,.3f);
    board_rx_info.is_pitch_mech_online=true;g.pitch.imu_angle=.7f;
    step(0);CHECK(!g.pitch.target_rebase_pending);near(g.pitch.target_imu_angle,.7f);
    saturate(660);CHECK(g.pitch.target_imu_angle>1.2f);
''')

    def test_sleep_and_mode_changes_drop_old_limit_reference(self):
        self.run_case(r'''
    saturate(660);g.mode=GIMBAL_MODE_SLEEP;step(660);
    near(g.pitch.target_imu_angle,g.pitch.imu_angle);
    g.mode=GIMBAL_MODE_GYRO;g.pitch.imu_angle=.7f;
    g.pitch.target_imu_angle=.7f;saturate(660);CHECK(g.pitch.target_imu_angle>1.2f);
    g.mode=GIMBAL_MODE_MECH;board_rx_info.pitch_mech_angle=.2f;step(0);
    near(g.pitch.target_mec_angle,.2f);
    saturate(660);near(g.pitch.target_mec_angle,g.pitch.max_limit);
    saturate(-660);near(g.pitch.target_mec_angle,g.pitch.min_limit);
    g.mode=GIMBAL_MODE_GYRO;board_rx_info.pitch_mech_angle=0;
    g.pitch.imu_angle=0;g.pitch.target_imu_angle=0;saturate(660);
    CHECK(g.pitch.target_imu_angle<.56f);
''')

    def test_keyboard_uses_same_boundary_without_replaying_blocked_input(self):
        self.run_case(r'''
    hero_car.input_src=INPUT_SRC_KEY;
    key_delta=2;step(0);float held=g.pitch.target_imu_angle;
    g.pitch.imu_angle+=.001f;
    key_delta=.1f;step(0);CHECK(g.pitch.target_imu_angle==held);
    key_delta=-.01f;step(0);near(g.pitch.target_imu_angle,held-.01f);
    step(0);near(g.pitch.target_imu_angle,held-.01f);CHECK(key_calls==4);
''')


if __name__ == '__main__':
    unittest.main()
