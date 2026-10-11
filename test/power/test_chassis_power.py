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

    def test_no_received_limit_does_not_fabricate_an_80w_allowance(self):
        self.run_case(r'''
    Chassis_Power_Update(&input,0);near(chassis_power.target_power_w,0);
    near(chassis_power.judge_limit_w,0);
    CHECK(!chassis_power.judge_limit_valid && chassis_power.fallback_active);
    limit_frame(60,10);Chassis_Power_Update(&input,10);
    near(chassis_power.base_budget_w,55);near(chassis_power.target_power_w,55);
    buffer_frame(60,600);Chassis_Power_Update(&input,600);
    CHECK(!chassis_power.judge_limit_valid && chassis_power.buffer_valid);
    near(chassis_power.base_budget_w,55);near(chassis_power.target_power_w,55);
    limit_frame(0,2001);Chassis_Power_Update(&input,2001);
    near(chassis_power.base_budget_w,55);CHECK(!chassis_power.judge_limit_valid);
''')

    def test_offline_training_requires_explicit_allowance(self):
        self.run_case(r'''
    near(chassis_power.training_limit_w,0);
    chassis_power.training_limit_w=80;Chassis_Power_Update(&input,0);
    near(chassis_power.target_power_w,75);near(chassis_power.judge_limit_w,0);
    CHECK(!chassis_power.judge_limit_valid);
    limit_frame(60,10);Chassis_Power_Update(&input,10);
    chassis_power.training_limit_w=120;Chassis_Power_Update(&input,1000);
    near(chassis_power.base_budget_w,55);
''')

    def test_first_joule_of_buffer_loss_derates_and_duplicate_frame_is_held(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(60,0);Chassis_Power_Update(&input,0);
    buffer_frame(59,100);Chassis_Power_Update(&input,100);
    near(chassis_power.buffer_derate_w,1);near(chassis_power.target_power_w,74);
    near(chassis_power.buffer_energy_j,59);near(input.buffer_j,59);
    near(chassis_power.buffer_drop_j,1);near(chassis_power.buffer_drop_rate_w,10);
    CHECK(chassis_power.buffer_drop_detected);
    for(uint32_t t=102;t<=198;t+=2)Chassis_Power_Update(&input,t);
    near(chassis_power.buffer_derate_w,1);near(chassis_power.target_power_w,74);
''')

    def test_zero_buffer_is_valid_and_higher_energy_never_buys_extra_power(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(0,0);Chassis_Power_Update(&input,0);
    CHECK(chassis_power.buffer_valid);near(chassis_power.buffer_derate_w,60);
    near(chassis_power.target_power_w,15);
    buffer_frame(250,100);Chassis_Power_Update(&input,100);
    CHECK(chassis_power.target_power_w<=75);near(input.buffer_j,250);
''')

    def test_warning_band_acts_before_ceiling_without_unbounded_accumulation(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(60,0);cap_frame(74,0);Chassis_Power_Update(&input,0);
    near(chassis_power.measured_derate_w,1);near(chassis_power.target_power_w,74);
    for(uint32_t t=100;t<=2000;t+=100){
        limit_frame(80,t);buffer_frame(60,t);cap_frame(74,t);Chassis_Power_Update(&input,t);
        near(chassis_power.measured_derate_w,1);near(chassis_power.target_power_w,74);
    }
''')

    def test_feedback_derates_choose_larger_instead_of_summing(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(59,0);cap_frame(74,0);Chassis_Power_Update(&input,0);
    near(chassis_power.buffer_derate_w,1);near(chassis_power.measured_derate_w,1);
    near(chassis_power.target_power_w,74);
    buffer_frame(55,100);cap_frame(75,100);Chassis_Power_Update(&input,100);
    near(chassis_power.buffer_derate_w,5);near(chassis_power.measured_derate_w,2);
    near(chassis_power.target_power_w,70);
''')

    def test_integration_uses_elapsed_time_not_number_of_frames(self):
        self.run_case(r'''
    float result[2];
    for(unsigned trial=0;trial<2;trial++){
        Chassis_Power_Init();input=(ChassisPowerInput_t){0};
        limit_frame(80,0);buffer_frame(60,0);cap_frame(85,0);Chassis_Power_Update(&input,0);
        uint32_t period=trial?50:200;
        for(uint32_t t=period;t<=1000;t+=period){
            limit_frame(80,t);buffer_frame(60,t);cap_frame(85,t);Chassis_Power_Update(&input,t);
        }
        result[trial]=chassis_power.measured_derate_w;
    }
    near(result[0],result[1]);near(result[0],15);
''')

    def test_repeated_and_reconnected_cap_frames_do_not_integrate_unknown_time(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(60,0);cap_frame(80,0);Chassis_Power_Update(&input,0);
    near(chassis_power.measured_derate_w,5);
    cap_frame(80,100);Chassis_Power_Update(&input,100);
    near(chassis_power.measured_derate_w,5.5f);
    for(uint32_t t=102;t<=198;t+=2)Chassis_Power_Update(&input,t);
    near(chassis_power.measured_derate_w,5.5f);
    cap_frame(80,100);Chassis_Power_Update(&input,200);
    near(chassis_power.measured_derate_w,5.5f);
    Chassis_Power_Update(&input,1000);CHECK(!chassis_power.cap_valid);
    near(chassis_power.measured_derate_w,5.5f);
    cap_frame(80,1100);Chassis_Power_Update(&input,1100);
    near(chassis_power.measured_derate_w,5.5f);
''')

    def test_recovery_requires_fresh_low_power_and_full_or_rising_buffer(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(60,0);cap_frame(80,0);Chassis_Power_Update(&input,0);
    limit_frame(80,100);buffer_frame(60,100);cap_frame(80,100);Chassis_Power_Update(&input,100);
    float held=chassis_power.measured_derate_w;
    for(uint32_t t=200;t<=800;t+=100){
        limit_frame(80,t);buffer_frame(59,t);cap_frame(50,t);Chassis_Power_Update(&input,t);
        near(chassis_power.measured_derate_w,held);
    }
    for(uint32_t t=900;t<=1300;t+=100){
        limit_frame(80,t);buffer_frame(60,t);cap_frame(50,t);Chassis_Power_Update(&input,t);
        near(chassis_power.measured_derate_w,held);
    }
    limit_frame(80,1400);buffer_frame(60,1400);cap_frame(50,1400);Chassis_Power_Update(&input,1400);
    CHECK(chassis_power.measured_derate_w<held && chassis_power.measured_derate_w>0);
    CHECK(chassis_power.target_power_w<=75);
    held=chassis_power.measured_derate_w;
    Chassis_Power_Update(&input,2000);near(chassis_power.measured_derate_w,held);
    limit_frame(80,2100);buffer_frame(60,2100);cap_frame(50,2100);Chassis_Power_Update(&input,2100);
    near(chassis_power.measured_derate_w,held);
''')

    def test_buffer_recovery_is_slow_and_reconnect_does_not_fake_drop_rate(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(40,0);cap_frame(50,0);Chassis_Power_Update(&input,0);
    near(chassis_power.buffer_derate_w,20);near(chassis_power.target_power_w,55);
    limit_frame(80,1000);buffer_frame(60,1000);cap_frame(50,1000);Chassis_Power_Update(&input,1000);
    near(chassis_power.buffer_derate_w,20);near(chassis_power.buffer_drop_rate_w,0);
    for(uint32_t t=1100;t<=1500;t+=100){
        limit_frame(80,t);buffer_frame(60,t);cap_frame(50,t);Chassis_Power_Update(&input,t);
    }
    CHECK(chassis_power.buffer_derate_w<20 && chassis_power.buffer_derate_w>0);
    CHECK(chassis_power.target_power_w>55 && chassis_power.target_power_w<=75);
    float held=chassis_power.buffer_derate_w;
    for(uint32_t t=1502;t<1600;t+=2)Chassis_Power_Update(&input,t);
    near(chassis_power.buffer_derate_w,held);
''')

    def test_timer_rollover_and_stale_first_limit_keep_last_known_value(self):
        self.run_case(r'''
    limit_frame(60,UINT32_MAX-100);Chassis_Power_Update(&input,UINT32_MAX-100);
    Chassis_Power_Update(&input,50);CHECK(chassis_power.judge_limit_valid);
    near(chassis_power.base_budget_w,55);
    Chassis_Power_Init();input=(ChassisPowerInput_t){0};
    limit_frame(50,0);Chassis_Power_Update(&input,1000);
    CHECK(!chassis_power.judge_limit_valid);near(chassis_power.base_budget_w,45);
''')

    def test_referee_limit_drop_is_immediate_and_rise_requires_confirmed_feedback(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(60,0);cap_frame(30,0);
    Chassis_Power_Update(&input,0);near(chassis_power.target_power_w,75);
    limit_frame(60,100);buffer_frame(60,100);cap_frame(30,100);
    Chassis_Power_Update(&input,100);near(chassis_power.target_power_w,55);
    limit_frame(80,200);buffer_frame(60,200);cap_frame(30,200);
    Chassis_Power_Update(&input,200);near(chassis_power.base_budget_w,75);
    near(chassis_power.target_power_w,55);CHECK(!chassis_power.recovery_ready);
    for(uint32_t t=300;t<=500;t+=100){
        limit_frame(80,t);buffer_frame(60,t);cap_frame(30,t);
        Chassis_Power_Update(&input,t);
    }
    CHECK(chassis_power.recovery_ready);near(chassis_power.target_power_w,57);
    Chassis_Power_Update(&input,502);near(chassis_power.target_power_w,57.04f);
    Chassis_Power_Update(&input,1000);near(chassis_power.target_power_w,57.04f);
    CHECK(!chassis_power.cap_valid && !chassis_power.recovery_ready);
''')

    def test_invalid_budget_parameter_clears_drive_and_restarts_recovery(self):
        self.run_case(r'''
    limit_frame(80,0);buffer_frame(60,0);cap_frame(30,0);
    Chassis_Power_Update(&input,0);
    chassis_power.measured_integral_gain_per_s=NAN;
    Chassis_Power_Update(&input,100);CHECK(chassis_power.config_fault);
    near(chassis_power.target_power_w,0);CHECK(!chassis_power.recovery_ready);
    float currents[4]={1000,-1000,1000,-1000},rpm[4]={0};
    Chassis_Power_Limit(currents,rpm);
    for(int i=0;i<4;i++)near(currents[i],0);
    chassis_power.measured_integral_gain_per_s=1;
    limit_frame(80,200);buffer_frame(60,200);cap_frame(30,200);
    Chassis_Power_Update(&input,200);CHECK(!chassis_power.config_fault);
    near(chassis_power.target_power_w,0);near(chassis_power.recovery_stable_ms,0);
    for(uint32_t t=300;t<=700;t+=100){
        limit_frame(80,t);buffer_frame(60,t);cap_frame(30,t);
        Chassis_Power_Update(&input,t);
    }
    CHECK(chassis_power.recovery_ready);near(chassis_power.target_power_w,2);
''')

    def test_all_four_currents_are_limited_without_sign_or_ratio_boost(self):
        self.run_case(r'''
    limit_frame(80,0);Chassis_Power_Update(&input,0);
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
    limit_frame(80,0);Chassis_Power_Update(&input,0);
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
    limit_frame(80,0);Chassis_Power_Update(&input,0);
    float currents[4]={10000,10000,10000,10000};float rpm[4]={9000,9000,9000,9000};
    Chassis_Power_Limit(currents,rpm);CHECK(chassis_power.budget_unreachable);
    for(int i=0;i<4;i++)near(currents[i],0);
    chassis_power.k_i2=NAN;
    currents[0]=10000;Chassis_Power_Limit(currents,rpm);
    CHECK(chassis_power.model_fault);for(int i=0;i<4;i++)near(currents[i],0);
''')

    def test_randomized_signed_motor_conditions_and_quantized_can_currents(self):
        self.run_case(r'''
    limit_frame(80,0);Chassis_Power_Update(&input,0);unsigned seed=123;
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
