"""Run real driver/task code with host CAN and sensor stubs."""
import re
import unittest
from test_ekf_init import ROOT, run_c


def function(path, signature):
    source = (ROOT / path).read_text(encoding='utf-8-sig')
    start = source.index(signature + '\n{')
    return source[start:source.index('\n}', start) + 2]


def driver():
    header = (ROOT / 'Application/HardwareLayer/DM_Motor.h').read_text(encoding='utf-8-sig')
    source = (ROOT / 'Application/HardwareLayer/DM_Motor.c').read_text(encoding='utf-8-sig')
    return r'''
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846f
#define __STM32F4xx_HAL_H
typedef int CAN_HandleTypeDef;
CAN_HandleTypeDef hcan2;
typedef enum {DEV_ONLINE, DEV_OFFLINE} dev_work_state_t;
static float constrain(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static unsigned sends;static uint8_t last_frame[8];
void CAN_SendData(CAN_HandleTypeDef *h,uint32_t id,uint8_t *buf){sends++;memcpy(last_frame,buf,8);}
''' + re.sub(r'^#include[^\n]*', '', header, flags=re.M) + '\n' + re.sub(
        r'^#include[^\n]*', '', source, flags=re.M)


class PitchFeedbackHealthTest(unittest.TestCase):
    def test_offline_safe_stop_requests_feedback_without_enabling(self):
        run_c(driver() + r'''
int main(void){
    Pitch_Motor.single_init(&Pitch_Motor);
    Pitch_Motor.single_sleep(&Pitch_Motor);
    assert(sends==1 && last_frame[7]==0xFD);
    assert(Pitch_Motor.state->status==DEV_OFFLINE);
    assert(Pitch_Motor.state->motor_state==Motor_Unenable);
    assert(Pitch_Motor.tx_info->torque==0 && Pitch_Motor.tx_info->Kp==0 && Pitch_Motor.tx_info->Kd==0);
}
''')

    def test_no_feedback_never_becomes_online(self):
        run_c(driver() + r'''
int main(void){
    Pitch_Motor.single_init(&Pitch_Motor);
    assert(Pitch_Motor.state->status==DEV_OFFLINE);
    for(int i=0;i<1000;i++){
        Pitch_Motor.single_heart_beat(&Pitch_Motor);
        assert(Pitch_Motor.state->status==DEV_OFFLINE);
    }
}
''')

    def test_receive_recovers_immediately_and_timeout_stays_offline(self):
        run_c(driver() + r'''
int main(void){
    uint8_t frame[8]={0x11,0x80,0,0x80,0,0,0,0};
    Pitch_Motor.single_init(&Pitch_Motor);
    for(int i=0;i<1000;i++)Pitch_Motor.single_heart_beat(&Pitch_Motor);
    Pitch_Motor.rx(&Pitch_Motor,frame);
    assert(Pitch_Motor.state->status==DEV_ONLINE);
    assert(Pitch_Motor.state->offline_cnt==0);
    for(int i=0;i<1000;i++){
        if(i%10==0)Pitch_Motor.rx(&Pitch_Motor,frame);
        Pitch_Motor.single_heart_beat(&Pitch_Motor);
        assert(Pitch_Motor.state->status==DEV_ONLINE);
    }
    for(int i=0;i<1000;i++)Pitch_Motor.single_heart_beat(&Pitch_Motor);
    assert(Pitch_Motor.state->status==DEV_OFFLINE);
    assert(Pitch_Motor.state->offline_cnt==Pitch_Motor.state->offline_cnt_max);
    Pitch_Motor.rx(&Pitch_Motor,frame);
    assert(Pitch_Motor.state->status==DEV_ONLINE);
}
''')

    def test_mechanical_frame_is_fresh_during_calibration_and_imu_fault(self):
        task = (ROOT / 'Application/TaskLayer/control_task.c').read_text(encoding='utf-8-sig')
        start = task.index('        Board_Comm_Check_Timeout();')
        body = task[start:task.index('        osDelay(2);', start)]
        safe_stop = function('Application/ModuleLayer/gimbal.c', 'void Gimbal_Safe_Stop(void)')
        send_state = function('Application/ProtocolLayer/board_comm.c',
                              'HAL_StatusTypeDef Board_Comm_Send_State(float pitch_mech_rad, bool pitch_valid)')
        header = (ROOT / 'Application/ProtocolLayer/board_comm.h').read_text(encoding='utf-8-sig')
        begin = header.index('typedef struct {', header.index('} Board_Rx_Info_t;'))
        tx_type = header[begin:header.index('} Board_Tx_Info_t;', begin)+len('} Board_Tx_Info_t;')]
        run_c(r'''
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#define IMU_NONE_ERR 0
#define IMU_DATA_CALI 1
#define DEV_ONLINE 0
#define DEV_OFFLINE 1
#define DEG_TO_RAD 0.0174532925f
#define PITCH_MECH_VALID_MARKER 0xA5U
#define MASTER_RX_STATE_CAN_ID 0x23
#define HAL_OK 0
typedef int HAL_StatusTypeDef;
int hcan2,send_result;
uint8_t sent[8];
int CAN_SendData(int*h,int id,uint8_t*buf){memcpy(sent,buf,8);return send_result;}
int CAN_SendData_Wait(int*h,int id,uint8_t*buf,uint32_t timeout){return CAN_SendData(h,id,buf);}
struct info {struct {float yaw,rate_yaw,pitch,rate_pitch;} base_info;} info;
struct sensor {struct {int err_code,cali_end;} work_state;struct info *info;void (*update)(struct sensor*);} imu_sensor;
struct {struct {float feedback_pitch,give_torque;} pitch;} gimbal;
struct state {int status;} state;
struct motor {struct state *state;void (*single_sleep)(struct motor*);} Pitch_Motor;
static float encoder_angle;static int works,stops;
void update(struct sensor*s){}
void Board_Comm_Check_Timeout(void){}
void Board_Comm_Send_IMU(float a,float b){}
void Board_Comm_Send_Pitch_IMU(float a,float b){}
void Gimbal_External_Date_Update(void){gimbal.pitch.feedback_pitch=encoder_angle;}
void Gimbal_Pitch_Clear_Pids(void){}
void sleep_motor(struct motor*m){stops++;}
void Gimbal_Work(void){Gimbal_External_Date_Update();works++;}
void Shoot_Work(void){}
''' + tx_type + '\nBoard_Tx_Info_t board_tx_info;\n' + send_state + '\n' + safe_stop +
              '\nvoid step(void){\n' + body + r'''
}
void check_angle(float expected){float actual;memcpy(&actual,sent+1,4);assert(fabsf(actual-expected)<1e-6f);}
int main(void){
    imu_sensor.info=&info;imu_sensor.update=update;
    Pitch_Motor.state=&state;Pitch_Motor.single_sleep=sleep_motor;
    state.status=DEV_ONLINE;
    imu_sensor.work_state.err_code=IMU_DATA_CALI;
    encoder_angle=.12f;step();
    assert(works==0 && stops==1 && sent[5]==0xA5);
    check_angle(.12f);
    imu_sensor.work_state.err_code=IMU_NONE_ERR;imu_sensor.work_state.cali_end=1;
    encoder_angle=.2f;step();assert(works==1 && sent[5]==0xA5);check_angle(.2f);
    imu_sensor.work_state.err_code=2;
    encoder_angle=.3f;step();assert(works==1 && stops==2 && sent[5]==0xA5);check_angle(.3f);
    state.status=DEV_OFFLINE;step();assert(sent[5]==0);
    state.status=DEV_ONLINE;encoder_angle=NAN;step();assert(sent[5]==0);
    encoder_angle=.3f;send_result=1;step();
    assert(board_tx_info.state_tx_fail_count==1 && board_tx_info.pitch_mech_valid);
    send_result=0;step();assert(board_tx_info.state_tx_fail_count==1);
}
''')


if __name__ == '__main__':
    unittest.main()
