"""Execute the firmware up-board pitch clamp with host stubs."""
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
    constants = '\n'.join(line for line in source.splitlines()
                          if re.match(r'#define\s+(DEG_TO_RAD|PITCH_LIMIT_)', line))
    start = source.index('static void Gimbal_Mech_Target_Update(void)\n{')
    update = source[start:source.index('\n}', start) + 2]
    return r'''
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
typedef struct { float kp; } pid_ctrl_t;
''' + constants + '\n' + pitch_type + r'''
typedef struct { Gimbal_Pitch_t pitch; } Gimbal_t;
struct { float pitch_encoder_tar; } board_rx_info;
static Gimbal_t gimbal;
''' + update + r'''
#define g gimbal
static void setup(void){
    gimbal=(Gimbal_t){0};
    g.pitch.use_imu=true;
    g.pitch.mec_min_rad=-0.314f;g.pitch.mec_max_rad=0.55f;
    board_rx_info.pitch_encoder_tar=0;
}
static void near(float a,float b){CHECK(fabsf(a-b)<1e-6f);}
'''


class PitchClampTest(unittest.TestCase):
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

    def test_hold_freezes_value_when_boundary_moves(self):
        self.run_case(r'''
    board_rx_info.pitch_encoder_tar=.6f;Gimbal_Mech_Target_Update();
    float held=g.pitch.target_pitch;
    CHECK(g.pitch.target_held);CHECK(g.pitch.held_at_max);
    /* 边界整体平移（机身倾斜）：贴界目标一个数位不动。 */
    g.pitch.imu_pitch=.05f;
    for(int i=0;i<100;i++){
        board_rx_info.pitch_encoder_tar=.6f+.001f*i;
        Gimbal_Mech_Target_Update();CHECK(g.pitch.target_pitch==held);
    }
''')

    def test_release_on_inward_reclamps_into_range(self):
        self.run_case(r'''
    board_rx_info.pitch_encoder_tar=.6f;Gimbal_Mech_Target_Update();
    float held=g.pitch.target_pitch;
    /* 边界移进旧目标内部：仍保持不动。 */
    g.pitch.imu_pitch=-.05f;
    Gimbal_Mech_Target_Update();CHECK(g.pitch.target_pitch==held);
    /* 目标向内移动即释放，并被截回当前可达区间。 */
    board_rx_info.pitch_encoder_tar=.3f;
    Gimbal_Mech_Target_Update();
    CHECK(!g.pitch.target_held);near(g.pitch.target_pitch,.3f);
    /* 释放后仍越界的：再截断一次并重新贴界给死。 */
    board_rx_info.pitch_encoder_tar=.9f;g.pitch.imu_pitch=.05f;
    Gimbal_Mech_Target_Update();
    CHECK(g.pitch.target_held);
    CHECK(g.pitch.target_pitch<=g.pitch.limit_offset+g.pitch.mec_max_rad-PITCH_LIMIT_MARGIN_RAD+1e-6f);
''')

    def test_hold_at_min_is_mirror_image(self):
        self.run_case(r'''
    board_rx_info.pitch_encoder_tar=-.6f;Gimbal_Mech_Target_Update();
    float held=g.pitch.target_pitch;
    CHECK(g.pitch.target_held);CHECK(!g.pitch.held_at_max);
    g.pitch.imu_pitch=.05f;
    for(int i=0;i<100;i++){
        board_rx_info.pitch_encoder_tar=-.6f-.001f*i;
        Gimbal_Mech_Target_Update();CHECK(g.pitch.target_pitch==held);
    }
    board_rx_info.pitch_encoder_tar=.3f;Gimbal_Mech_Target_Update();
    CHECK(!g.pitch.target_held);near(g.pitch.target_pitch,.3f);
''')

    def test_offset_jitter_under_hysteresis_does_not_move_bound(self):
        self.run_case(r'''
    board_rx_info.pitch_encoder_tar=.6f;Gimbal_Mech_Target_Update();
    float held=g.pitch.target_pitch;
    /* 采样时差噪声小于锁存阈值：边界不动，截断结果也不动。 */
    for(int i=0;i<1000;i++){
        g.pitch.imu_pitch=(i%2 ? .002f : -.002f);
        board_rx_info.pitch_encoder_tar=.6f+(i%2 ? .001f : -.001f);
        Gimbal_Mech_Target_Update();CHECK(g.pitch.target_pitch==held);
    }
''')

    def test_interior_target_is_not_dragged_by_feedback(self):
        self.run_case(r'''
    board_rx_info.pitch_encoder_tar=.2f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,.2f);CHECK(!g.pitch.target_held);
    g.pitch.imu_pitch=.05f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,.2f);
''')

    def test_mech_mode_uses_constant_limits(self):
        self.run_case(r'''
    g.pitch.use_imu=false;
    board_rx_info.pitch_encoder_tar=.9f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,g.pitch.mec_max_rad);
    board_rx_info.pitch_encoder_tar=-.9f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,g.pitch.mec_min_rad);
    board_rx_info.pitch_encoder_tar=.1f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,.1f);CHECK(!g.pitch.target_held);
''')


if __name__ == '__main__':
    unittest.main()
