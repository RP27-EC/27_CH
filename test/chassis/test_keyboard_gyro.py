"""Exercise keyboard mapping, chassis following and mode-specific alignment."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def read(name):
    return (ROOT / name).read_text(encoding='utf-8-sig')


def strip_includes(source):
    return re.sub(r'^#include[^\n]*', '', source, flags=re.M)


def function(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start) + 2]


def harness():
    gimbal = read('Application/ModuleLayer/gimbal.c')
    start = gimbal.index('    if ((car_mode == CAR_MODE_SPIN) || gimbal->yaw.turn_active)')
    end = gimbal.index('    switch (gimbal->mode)', start)
    return r'''
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#undef errno /* Windows CRT 宏与固件 rc_sensor.errno 字段同名。 */
#define PI 3.14159265358979323846f
#define constrain(x,lo,hi) ((x)>(hi)?(hi):((x)<(lo)?(lo):(x)))
#define CHECK(x) do{if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
''' + read('Application/ControlLayer/car_control.h') + read('Application/AlgorithmLayer/PID.h') + strip_includes(read('Application/ModuleLayer/gimbal.h')) + strip_includes(read('Application/ModuleLayer/chassis.h')) + read('Application/ModuleLayer/chassis_kinematics.h') + read('Application/AlgorithmLayer/chassis_power.h') + '\n'.join(re.findall(r'^#define CHASSIS_(?:FL|FR|BL|BR)_DIRECTION[^\n]*', read('Application/DeviceLayer/chassis_motor.h'), re.M)) + '\n' + '\n'.join(re.findall(r'^#define KEY_[WAQ]_CNT_MAX[^\n]*', read('Application/DeviceLayer/Sensor/rc_sensor.h'), re.M)) + r'''
#include <string.h>
static uint32_t tick;
static uint32_t irq_mask;
static void (*after_irq_restore)(void);
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return irq_mask;}
static void __disable_irq(void){irq_mask=1;}
static void __set_PRIMASK(uint32_t x){irq_mask=x;if(after_irq_restore)after_irq_restore();}
struct {
    struct {struct {uint16_t chassis_power_limit;} game_robot_status;
            struct {uint16_t chassis_power_buffer;} power_heat_data;} *org_info;
    struct {bool limit_received,buffer_received;
            uint32_t limit_rx_count,limit_update_ms,buffer_rx_count,buffer_update_ms;} *status;
} My_Judge;
struct {
    struct {int16_t chassis_power;} *info;
    struct {bool power_received;uint32_t power_rx_count,power_update_ms;} *status;
} cap;
enum {release, release_to_press};
enum {DEV_OFFLINE, DEV_ONLINE, NONE_ERR, DEV_DATA_ERR};
struct Key {int cnt,value,status;};
struct Rc {struct Key W,A,S,D,Q,E,Ctrl,Shift;int16_t ch0,ch2,ch3,mouse_vx;
    float mouse_x;uint16_t frame_seq;uint32_t frame_update_ms,frame_interval_ms;
    bool frame_received;} rc;
struct {struct Rc *info;int work_state,errno;} rc_sensor={&rc,DEV_ONLINE,NONE_ERR};
struct {bool is_online;} board_rx_info={true};
static bool ctrl_fall_edge;
static uint8_t ctrl_val_snap;
Car_t hero_car;
Gimbal_t hero_gimbal;
typedef struct Motor Motor;
struct Motor {
    struct {pid_ctrl_t *speed_ctrl;} *ctrl;
    struct {float torque,torque_current;int16_t torque_current_raw;} *tx_info;
    struct {int16_t encoder_speed;} *rx_info;
    void (*single_set_speed)(Motor *);
};
static pid_ctrl_t wheel_pid[4];
static Motor Chassis_Motor[4];
struct Group {void (*group_set_torque)(struct Group *);};
static struct Group Chassis_Motor_Group;
static float sent_current[4];
static void group_send(struct Group *g){(void)g;for(int i=0;i<4;i++)sent_current[i]=Chassis_Motor[i].tx_info->torque;}
void pid_clear(pid_ctrl_t *p){*p=(pid_ctrl_t){0};}
static void motor_set_speed(Motor *m){m->tx_info->torque=m->ctrl->speed_ctrl->target*3;}
''' + strip_includes(read('Application/AlgorithmLayer/chassis_power.c')) + strip_includes(read('Application/ModuleLayer/chassis_power_adapter.c')) + strip_includes(read('Application/ModuleLayer/chassis_kinematics.c')) + strip_includes(read('Application/ModuleLayer/chassis.c')) + function(read('Application/ControlLayer/carctrl.c'), 'void Car_Chassis_Channel_Get(') + '\nstatic void update_alignment(Gimbal_t *gimbal,CarMode_e car_mode){\n' + gimbal[start:end] + '\n}\n' + function(gimbal, 'static void Gimbal_Mech_Update(') + function(gimbal, 'static float Gimbal_Trim_Step(') + function(gimbal, 'static void Gimbal_Key_Yaw_Feedforward_Clear(') + function(gimbal, 'static void Gimbal_Key_Yaw_Feedforward_Enable(') + function(gimbal, 'static void Gimbal_Key_Yaw_Update(') + r'''
static void setup(void){
    My_Judge.org_info=calloc(1,sizeof(*My_Judge.org_info));
    My_Judge.status=calloc(1,sizeof(*My_Judge.status));
    cap.info=calloc(1,sizeof(*cap.info));cap.status=calloc(1,sizeof(*cap.status));
    Chassis_Motor_Group.group_set_torque=group_send;
    for(int i=0;i<4;i++){
        Chassis_Motor[i].ctrl=calloc(1,sizeof(*Chassis_Motor[i].ctrl));
        Chassis_Motor[i].tx_info=calloc(1,sizeof(*Chassis_Motor[i].tx_info));
        Chassis_Motor[i].rx_info=calloc(1,sizeof(*Chassis_Motor[i].rx_info));
        Chassis_Motor[i].ctrl->speed_ctrl=&wheel_pid[i];
        Chassis_Motor[i].single_set_speed=motor_set_speed;
    }
    Chassis_Power_Init();
    hero_gimbal.yaw.gyro_sensitivity=.000015f;hero_gimbal.yaw.key_yaw_ff_gain=1.0f;
}
static void near(float a,float b){CHECK(fabsf(a-b)<.001f);}
'''


class KeyboardGyroTest(unittest.TestCase):
    def run_case(self, body, preamble=''):
        with tempfile.TemporaryDirectory() as directory:
            src = pathlib.Path(directory) / 'keyboard_gyro.c'
            exe = pathlib.Path(directory) / 'keyboard_gyro.exe'
            src.write_text(harness() + preamble + '\nint main(void){setup();\n' + body + '\nreturn 0;}\n', encoding='utf-8')
            compiled = subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', str(src), '-lm', '-o', str(exe)], capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_a_left_d_right_in_mech_and_gyro(self):
        self.run_case(r'''
    for(int mode=0;mode<2;mode++){
        Car_t car={0};car.input_src=INPUT_SRC_KEY;
        car.mode=mode?CAR_MODE_GYRO:CAR_MODE_MECH;
        int16_t f,l,r;
        rc.A.cnt=KEY_A_CNT_MAX;rc.D.cnt=0;
        Car_Chassis_Channel_Get(&car,&f,&l,&r);CHECK(l==-660);
        Chassis_Update(f,l,r,0,ALIGN_FRONT,car.mode,false);
        CHECK(wheel_pid[0].target<0 && wheel_pid[2].target>0);
        rc.A.cnt=0;rc.D.cnt=KEY_A_CNT_MAX;
        Car_Chassis_Channel_Get(&car,&f,&l,&r);CHECK(l==660);
        Chassis_Update(f,l,r,0,ALIGN_FRONT,car.mode,false);
        CHECK(wheel_pid[0].target>0 && wheel_pid[2].target<0);
    }
''')

    def test_rc_mapping_and_ctrl_suppression_remain(self):
        self.run_case(r'''
    Car_t car={0};car.mode=CAR_MODE_GYRO;car.input_src=INPUT_SRC_RC;
    rc.ch3=111;rc.ch2=222;rc.ch0=333;int16_t f,l,r;
    Car_Chassis_Channel_Get(&car,&f,&l,&r);CHECK(f==111 && l==222 && r==333);
    car.input_src=INPUT_SRC_KEY;ctrl_val_snap=1;
    rc.A.cnt=KEY_A_CNT_MAX;Car_Chassis_Channel_Get(&car,&f,&l,&r);
    CHECK(f==0 && l==0 && r==0);
''')

    def test_gyro_follows_front_even_after_mechanical_back_lock(self):
        self.run_case(r'''
    float front[4];
    const float angles[]={2.8f,-2.8f};
    for(int j=0;j<2;j++){
        Chassis_Update(0,0,0,angles[j],ALIGN_FRONT,CAR_MODE_GYRO,false);
        for(int i=0;i<4;i++)front[i]=wheel_pid[i].target;
        Chassis_Update(0,0,0,angles[j],ALIGN_BACK,CAR_MODE_GYRO,false);
        for(int i=0;i<4;i++)near(wheel_pid[i].target,front[i]);
    }
''')

    def test_only_mechanical_mode_updates_nearest_alignment(self):
        self.run_case(r'''
    Gimbal_t g={0};g.yaw.feedback_mec_angle=2.8f;
    g.yaw.align_dir=ALIGN_FRONT;g.yaw.target_imu_angle=.7f;
    update_alignment(&g,CAR_MODE_GYRO);
    CHECK(g.yaw.align_dir==ALIGN_FRONT);near(g.yaw.target_imu_angle,.7f);
    g.yaw.align_dir=ALIGN_BACK;g.yaw.feedback_mec_angle=.1f;
    update_alignment(&g,CAR_MODE_GYRO);CHECK(g.yaw.align_dir==ALIGN_BACK);
    update_alignment(&g,CAR_MODE_MECH);CHECK(g.yaw.align_dir==ALIGN_FRONT);
    g.yaw.feedback_mec_angle=2.8f;update_alignment(&g,CAR_MODE_MECH);
    CHECK(g.yaw.align_dir==ALIGN_BACK);Gimbal_Mech_Update(&g);
    near(g.yaw.target_mec_angle,PI);
''')


if __name__ == '__main__':
    unittest.main()
