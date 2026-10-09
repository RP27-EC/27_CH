"""Run firmware target handling and task scheduling with host stubs."""
import unittest
from test_ekf_init import ROOT, run_c


class PitchControlTest(unittest.TestCase):
    def test_upward_motion_has_consistent_local_and_transmitted_feedback(self):
        source = (ROOT / 'Application/ModuleLayer/gimbal.c').read_text(encoding='utf-8-sig')
        assignments = '\n'.join(line for line in source.splitlines()
                                if 'gimbal.pitch.imu_' in line and 'imu_sensor.info->' in line)
        task = (ROOT / 'Application/TaskLayer/control_task.c').read_text(encoding='utf-8-sig')
        start = task.index('Board_Comm_Send_Pitch_IMU(')
        send = task[start:task.index(';', start) + 1]
        run_c(r'''
#include <assert.h>
#include <math.h>
#define DEG_TO_RAD 0.01745329251994329577f
struct info {struct {float pitch,rate_pitch;} base_info;} info;
struct {struct info *info;} imu_sensor;
struct {struct {float imu_pitch,imu_speed;} pitch;} gimbal;
float sent_angle,sent_rate;
void Board_Comm_Send_Pitch_IMU(float angle,float rate){sent_angle=angle;sent_rate=rate;}
void step(void){
''' + assignments + '\n' + send + r'''
}
int main(void){
    imu_sensor.info=&info;
    /* Synthetic upward motion using the sign confirmed on the bench. */
    info.base_info.pitch=10;info.base_info.rate_pitch=10;step();
    float previous=gimbal.pitch.imu_pitch;
    info.base_info.pitch=11;step();
    assert(gimbal.pitch.imu_pitch>previous);
    assert(fabsf((gimbal.pitch.imu_pitch-previous)/.1f-gimbal.pitch.imu_speed)<1e-5f);
    assert(sent_angle==gimbal.pitch.imu_pitch && sent_rate==gimbal.pitch.imu_speed);
    info.base_info.pitch=10;info.base_info.rate_pitch=-10;step();
    assert(sent_rate<0 && sent_angle==gimbal.pitch.imu_pitch);
    info.base_info.rate_pitch=0;step();assert(sent_rate==0);
}
''')

    def test_targets_use_the_current_limits_for_their_domain(self):
        source = (ROOT / 'Application/ModuleLayer/gimbal.c').read_text(encoding='utf-8-sig')
        start = source.index('static void Gimbal_Mech_Target_Update(void)\n{')
        body = source[start:source.index('\n}', start) + 2]
        header = (ROOT / 'Application/ModuleLayer/gimbal.h').read_text(encoding='utf-8-sig')
        limits = '\n'.join(line for line in header.splitlines()
                           if line.startswith('#define PITCH_MIN_IMU_ANGLE') or
                           line.startswith('#define PITCH_MAX_IMU_ANGLE'))
        run_c(limits + '\n' + r'''
#include <stdbool.h>
#include <assert.h>
struct { struct { float target_pitch,imu_min_rad,imu_max_rad,mec_min_rad,mec_max_rad; bool use_imu; } pitch; } gimbal;
struct {float pitch_encoder_tar;} board_rx_info;
''' + body + r'''
int main(void){
    gimbal.pitch.imu_min_rad=0;gimbal.pitch.imu_max_rad=.68f;
    gimbal.pitch.mec_min_rad=-.3f;gimbal.pitch.mec_max_rad=.55f;
    gimbal.pitch.use_imu=true;
    board_rx_info.pitch_encoder_tar=-.2f;Gimbal_Mech_Target_Update();
    assert(gimbal.pitch.target_pitch==PITCH_MIN_IMU_ANGLE);
    board_rx_info.pitch_encoder_tar=.8f;Gimbal_Mech_Target_Update();
    assert(gimbal.pitch.target_pitch==PITCH_MAX_IMU_ANGLE);
    gimbal.pitch.use_imu=false;Gimbal_Mech_Target_Update();
    assert(gimbal.pitch.target_pitch==.55f);
    board_rx_info.pitch_encoder_tar=-.8f;Gimbal_Mech_Target_Update();
    assert(gimbal.pitch.target_pitch==-.3f);
}
''')

    def test_calibration_updates_without_enabling_and_rechecks_update_status(self):
        source = (ROOT / 'Application/TaskLayer/control_task.c').read_text(encoding='utf-8-sig')
        start = source.index('        Board_Comm_Check_Timeout();')
        body = source[start:source.index('        osDelay(2);', start)]
        run_c(r'''
#include <stdbool.h>
#include <assert.h>
#define IMU_NONE_ERR 0
#define IMU_DATA_CALI 1
#define DEV_ONLINE 0
#define DEG_TO_RAD 0.0174532925f
struct info {struct {float yaw,rate_yaw,pitch,rate_pitch;} base_info;} info;
struct sensor {struct {int err_code,cali_end;} work_state;struct info *info;void (*update)(struct sensor*);} imu_sensor;
struct {struct {float feedback_pitch;} pitch;} gimbal;
struct motor_state {int status;} motor_state;
struct {struct motor_state *state;} Pitch_Motor={&motor_state};
static int updates,works,stops,transition;static bool valid;
void update(struct sensor*s){updates++;if(transition==1){s->work_state.err_code=0;s->work_state.cali_end=1;}if(transition==2)s->work_state.err_code=2;}
void Board_Comm_Check_Timeout(void){}
void Board_Comm_Send_IMU(float a,float b){}
void Board_Comm_Send_Pitch_IMU(float a,float b){}
void Board_Comm_Send_State(float a,bool b){valid=b;}
void Gimbal_Work(void){works++;}
void Gimbal_External_Date_Update(void){}
void Gimbal_Safe_Stop(void){stops++;}
void Shoot_Work(void){}
void step(void){
''' + body + r'''
}
int main(void){
    imu_sensor.info=&info;imu_sensor.update=update;
    imu_sensor.work_state.err_code=IMU_DATA_CALI;
    step();assert(updates==1 && works==0 && stops==1 && valid);
    imu_sensor.work_state.err_code=IMU_NONE_ERR;
    step();assert(updates==2 && works==0 && valid);
    imu_sensor.work_state.err_code=IMU_DATA_CALI;transition=1;
    step();assert(updates==3 && works==1 && valid);
    transition=2;step();assert(updates==4 && works==1 && valid);
    step();assert(updates==4 && works==1 && valid);
    motor_state.status=1;step();assert(!valid && works==1);
}
''')
