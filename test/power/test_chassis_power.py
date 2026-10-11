"""Run the actual C power-budget and four-current limiter on the host."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
HEADER = ROOT / 'Application/AlgorithmLayer/chassis_power.h'
SOURCE = ROOT / 'Application/AlgorithmLayer/chassis_power.c'

PREAMBLE = r'''
#include "chassis_power.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) {fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
static void near(float a,float b) {CHECK(isfinite(a)&&fabsf(a-b)<.001f);}
static ChassisPowerInput_t input;
static void limit_frame(float watts,uint32_t t) {
    input.limit_received=true;input.limit_seq++;
    input.limit_ms=t;input.limit_w=watts;
}
static void buffer_frame(float joules,uint32_t t) {
    input.buffer_received=true;input.buffer_seq++;
    input.buffer_ms=t;input.buffer_j=joules;
}
static void cap_frame(float watts,uint32_t t) {
    input.cap_received=true;input.cap_seq++;
    input.cap_ms=t;input.cap_w=watts;
}
int main(void) {
    Chassis_Power_Init();
'''


class ChassisPowerTest(unittest.TestCase):
    def run_case(self, body):
        self.assertTrue(HEADER.exists() and SOURCE.exists(),
                        'The agreed chassis power budget/limiter has not been implemented')
        with tempfile.TemporaryDirectory() as directory:
            src = pathlib.Path(directory) / 'power_test.c'
            exe = pathlib.Path(directory) / 'power_test.exe'
            src.write_text(PREAMBLE + body + '\nreturn 0;}\n', encoding='utf-8')
            built = subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                                    '-Wno-unused-function', '-I' + str(HEADER.parent),
                                    str(src), str(SOURCE), '-lm', '-o', str(exe)],
                                   capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            run = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_training_last_limit_and_independent_timeout(self):
        self.run_case(r'''
    Chassis_Power_Update(&input,0);near(chassis_power.target_power_w,75);
    CHECK(!chassis_power.judge_limit_valid && chassis_power.fallback_active);
    limit_frame(60,10);Chassis_Power_Update(&input,10);
    near(chassis_power.target_power_w,55);CHECK(chassis_power.judge_limit_valid);
    buffer_frame(60,600);Chassis_Power_Update(&input,600);
    CHECK(!chassis_power.judge_limit_valid && chassis_power.buffer_valid);
    near(chassis_power.base_budget_w,55);near(chassis_power.target_power_w,55);
    Chassis_Power_Update(&input,2000);near(chassis_power.target_power_w,55);
    CHECK(!chassis_power.buffer_valid);near(chassis_power.last_valid_limit_w,60);
    limit_frame(0,2001);Chassis_Power_Update(&input,2001);
    near(chassis_power.base_budget_w,55);CHECK(!chassis_power.judge_limit_valid);
''')

    def test_buffer_only_derates_freezes_when_stale_and_recovers_slowly(self):
        self.run_case(r'''
    Chassis_Power_Update(&input,0);
    buffer_frame(60,100);Chassis_Power_Update(&input,100);
    near(chassis_power.target_power_w,75);
    buffer_frame(40,200);Chassis_Power_Update(&input,200);
    near(chassis_power.buffer_derate_w,10);near(chassis_power.target_power_w,65);
    near(chassis_power.buffer_drop_j,20);near(chassis_power.buffer_drop_rate_w,200);
    CHECK(chassis_power.buffer_drop_detected);
    Chassis_Power_Update(&input,202);
    near(chassis_power.buffer_drop_j,20);CHECK(chassis_power.buffer_drop_detected);
    Chassis_Power_Update(&input,2000);
    near(chassis_power.buffer_derate_w,10);near(chassis_power.target_power_w,65);
    buffer_frame(60,2100);Chassis_Power_Update(&input,2100);
    CHECK(chassis_power.buffer_derate_w>=9 && chassis_power.buffer_derate_w<10);
    CHECK(chassis_power.target_power_w>65 && chassis_power.target_power_w<=66);
    near(chassis_power.buffer_drop_j,0);
    float held=chassis_power.buffer_derate_w;
    for(uint32_t t=2102;t<2200;t+=2)Chassis_Power_Update(&input,t);
    near(chassis_power.buffer_derate_w,held);
''')

    def test_zero_buffer_is_valid_and_cannot_increase_budget(self):
        self.run_case(r'''
    buffer_frame(0,0);Chassis_Power_Update(&input,0);
    CHECK(chassis_power.buffer_valid);near(chassis_power.buffer_derate_w,50);
    near(chassis_power.target_power_w,25);
    buffer_frame(250,100);Chassis_Power_Update(&input,100);
    CHECK(chassis_power.target_power_w<=75);
''')

    def test_measurement_updates_once_per_frame_and_stale_correction_is_held(self):
        self.run_case(r'''
    cap_frame(85,0);Chassis_Power_Update(&input,0);
    near(chassis_power.measured_derate_w,10);near(chassis_power.target_power_w,65);
    for(uint32_t t=2;t<=100;t+=2)Chassis_Power_Update(&input,t);
    near(chassis_power.measured_derate_w,10);
    Chassis_Power_Update(&input,1000);CHECK(!chassis_power.cap_valid);
    near(chassis_power.measured_derate_w,10);
    cap_frame(50,1100);Chassis_Power_Update(&input,1100);
    CHECK(chassis_power.measured_derate_w>=9.9f && chassis_power.measured_derate_w<10);
    CHECK(chassis_power.target_power_w<=75);
''')

    def test_timer_rollover_and_stale_first_limit_keep_last_known_value(self):
        self.run_case(r'''
    limit_frame(60,UINT32_MAX-100);Chassis_Power_Update(&input,UINT32_MAX-100);
    Chassis_Power_Update(&input,50);CHECK(chassis_power.judge_limit_valid);
    near(chassis_power.base_budget_w,55);
    Chassis_Power_Init();limit_frame(50,0);Chassis_Power_Update(&input,1000);
    CHECK(!chassis_power.judge_limit_valid);near(chassis_power.base_budget_w,45);
''')

    def test_all_four_currents_are_limited_without_sign_or_ratio_boost(self):
        self.run_case(r'''
    Chassis_Power_Update(&input,0);
    float currents[4]={10000,-10000,8000,-8000};
    float rpm[4]={3000,-3000,2500,-2500};
    Chassis_Power_Limit(currents,rpm);
    CHECK(chassis_power.current_scale>0 && chassis_power.current_scale<1);
    CHECK(chassis_power.predicted_before_w>75);
    CHECK(chassis_power.predicted_after_w<=75.001f);
    CHECK(currents[0]>0 && currents[1]<0 && currents[2]>0 && currents[3]<0);
    CHECK(fabsf(currents[0])<=10000 && fabsf(currents[2])<=8000);
    near(currents[0],-currents[1]);near(currents[2],-currents[3]);
    CHECK(fabsf(currents[2]/currents[0]-.8f)<.001f);
''')

    def test_under_budget_preserves_current_and_braking_does_not_buy_power(self):
        self.run_case(r'''
    Chassis_Power_Update(&input,0);
    float currents[4]={100,-100,100,-100};float rpm[4]={100,-100,100,-100};
    Chassis_Power_Limit(currents,rpm);near(chassis_power.current_scale,1);
    near(currents[0],100);near(currents[1],-100);
    currents[0]=10000;currents[1]=-2000;currents[2]=currents[3]=0;
    rpm[0]=rpm[1]=3000;rpm[2]=rpm[3]=0;
    Chassis_Power_Limit(currents,rpm);
    CHECK(chassis_power.predicted_after_w<=75.001f);
    CHECK(currents[0]>=0 && currents[1]<=0);
''')

    def test_impossible_baseline_and_nonfinite_model_do_not_release_original_current(self):
        self.run_case(r'''
    Chassis_Power_Update(&input,0);
    float currents[4]={10000,10000,10000,10000};float rpm[4]={9000,9000,9000,9000};
    Chassis_Power_Limit(currents,rpm);CHECK(chassis_power.budget_unreachable);
    for(int i=0;i<4;i++)near(currents[i],0);
    chassis_power.k_i2=NAN;
    currents[0]=10000;Chassis_Power_Limit(currents,rpm);
    CHECK(chassis_power.model_fault);for(int i=0;i<4;i++)near(currents[i],0);
''')

    def test_randomized_signed_motor_conditions_and_quantized_can_currents(self):
        self.run_case(r'''
    Chassis_Power_Update(&input,0);unsigned seed=123;
    for(int trial=0;trial<2000;trial++){
        float currents[4],original[4],rpm[4];
        for(int i=0;i<4;i++){
            seed=seed*1664525U+1013904223U;currents[i]=(int)(seed%32769)-16384;
            original[i]=currents[i];
            seed=seed*1664525U+1013904223U;rpm[i]=(int)(seed%10001)-5000;
        }
        Chassis_Power_Limit(currents,rpm);
        CHECK(!chassis_power.model_fault && !chassis_power.budget_unreachable);
        CHECK(chassis_power.predicted_after_w<=75.001f);
        for(int i=0;i<4;i++){
            CHECK(fabsf(currents[i])<=fabsf(original[i]));
            CHECK(currents[i]*original[i]>=0);
            near(currents[i],(float)(int16_t)currents[i]);
        }
    }
''')


if __name__ == '__main__':
    unittest.main()
