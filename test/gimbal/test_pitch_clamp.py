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
    constants = '\n'.join(line for line in header.splitlines()
                          if re.match(r'#define\s+PITCH_', line))
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
    gimbal.pitch.use_imu=true;
    gimbal.pitch.mec_min_rad=-0.314f;gimbal.pitch.mec_max_rad=0.55f;
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

    def test_use_imu_clamps_to_static_imu_limits(self):
        self.run_case(r'''
    board_rx_info.pitch_encoder_tar=.9f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,PITCH_MAX_IMU_ANGLE);
    board_rx_info.pitch_encoder_tar=-.9f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,PITCH_MIN_IMU_ANGLE);
    board_rx_info.pitch_encoder_tar=.3f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,.3f);
''')

    def test_limit_is_flat_despite_feedback_jitter(self):
        for expr, bound in (('PITCH_MAX_IMU_ANGLE+.3f', 'PITCH_MAX_IMU_ANGLE'),
                            ('PITCH_MIN_IMU_ANGLE-.3f', 'PITCH_MIN_IMU_ANGLE')):
            with self.subTest(expr=expr):
                self.run_case(
                    f'float tar={expr};board_rx_info.pitch_encoder_tar=tar;\n'
                    f'    Gimbal_Mech_Target_Update();\n'
                    f'    float held=g.pitch.target_pitch;\n'
                    f'    near(held,{bound});\n'  # 边界必须就是静态常数本身，不是别的平坦值
                    '    float outward=(tar>held ? .3f : -.3f);\n'
                    r'''    /* IMU/编码器反馈怎么抖，边界都是常数，截断结果一个数位不动。 */
    for(int i=0;i<1000;i++){
        float jitter=(i%2 ? .002f : -.002f);
        g.pitch.imu_pitch=jitter;g.pitch.feedback_pitch=-jitter;
        board_rx_info.pitch_encoder_tar=held+outward+(i%2 ? .01f : -.01f);
        Gimbal_Mech_Target_Update();CHECK(g.pitch.target_pitch==held);
    }
''')

    def test_reverse_leaves_limit_on_first_call(self):
        """反向输入当拍离开限位：不得有任何锁存/保持把目标钉在边界上。"""
        for expr, bound, inward in (('PITCH_MAX_IMU_ANGLE+.3f', 'PITCH_MAX_IMU_ANGLE', '-.01f'),
                                    ('PITCH_MIN_IMU_ANGLE-.3f', 'PITCH_MIN_IMU_ANGLE', '+.01f')):
            with self.subTest(expr=expr):
                self.run_case(
                    f'float tar={expr};board_rx_info.pitch_encoder_tar=tar;\n'
                    '    Gimbal_Mech_Target_Update();\n'
                    f'    near(g.pitch.target_pitch,{bound});\n'
                    f'    board_rx_info.pitch_encoder_tar={bound}{inward};\n'
                    '    Gimbal_Mech_Target_Update();\n'
                    f'    near(g.pitch.target_pitch,{bound}{inward});\n')

    def test_mech_mode_uses_encoder_range(self):
        self.run_case(r'''
    g.pitch.use_imu=false;
    board_rx_info.pitch_encoder_tar=.9f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,g.pitch.mec_max_rad);
    board_rx_info.pitch_encoder_tar=-.9f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,g.pitch.mec_min_rad);
    board_rx_info.pitch_encoder_tar=.1f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,.1f);
''')

    def test_interior_target_is_not_dragged_by_feedback(self):
        self.run_case(r'''
    board_rx_info.pitch_encoder_tar=.4f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,.4f);
    g.pitch.imu_pitch=.3f;g.pitch.feedback_pitch=-.2f;
    board_rx_info.pitch_encoder_tar=.4f;Gimbal_Mech_Target_Update();
    near(g.pitch.target_pitch,.4f);
''')


if __name__ == '__main__':
    unittest.main()
