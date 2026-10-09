"""Execute the firmware initializer with a host math/sensor harness."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def run_c(source):
    with tempfile.TemporaryDirectory() as directory:
        c = pathlib.Path(directory) / 'check.c'
        exe = pathlib.Path(directory) / 'check.exe'
        c.write_text(source, encoding='utf-8')
        subprocess.run(['gcc', '-std=c99', str(c), '-lm', '-o', str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(result.stderr or f'exit {result.returncode}')


class InitialQuaternionTest(unittest.TestCase):
    def test_time_wrap_duplicate_and_long_gap(self):
        source = (ROOT / 'Application/DeviceLayer/Sensor/imu_sensor.c').read_text(encoding='utf-8-sig')
        start = source.index('    imu_tick_now = micros();')
        end = source.index('    /* 从 EKF 结构体中提取结果 */', start)
        body = source[start:end]
        run_c(r'''
#include <stdint.h>
#include <math.h>
#include <assert.h>
static uint32_t now,imu_tick_now,imu_tick_last;
static uint8_t imu_tick_initialized;
static float imu_dt,gyrox,gyroy,gyroz,accx,accy,accz,last_dt;
static int calls;
uint32_t micros(void){return now;}
void IMU_QuaternionEKF_Update(float x,float y,float z,float a,float b,float c,float dt){
    calls++;last_dt=dt;
}
void step(void){
''' + body + r'''
}
int main(void){
    now=0;step();assert(calls==1 && fabsf(last_dt-0.002f)<1e-7f);
    step();assert(calls==1);
    now=2000;step();assert(calls==2);
    now+=20000;step();assert(calls==2);
    now+=2000;step();assert(calls==3);
    imu_tick_last=UINT32_MAX-999u;now=1000;step();
    assert(calls==4 && fabsf(last_dt-0.002f)<1e-7f);
    return 0;
}
''')

    def test_invalid_samples_are_rejected(self):
        source = (ROOT / 'Application/DeviceLayer/Imu/bmi_EKF.c').read_text(encoding='utf-8-sig')
        start = source.index('void IMU_QuaternionEKF_Update(')
        end = source.index('    /* 临时变量', start)
        run_c('#include <math.h>\n#include <assert.h>\nstatic int calls;\n' + source[start:end] + r'''
    calls++;
}
int main(void){
    IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,0.002f);assert(calls==1);
    IMU_QuaternionEKF_Update(0,0,0,0,0,0,0.002f);
    IMU_QuaternionEKF_Update(NAN,0,0,0,0,9.8f,0.002f);
    IMU_QuaternionEKF_Update(0,0,0,0,0,INFINITY,0.002f);
    IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,0);
    IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,0.02f);
    assert(calls==1);return 0;
}
''')

    def test_bias_feedback_and_mahony_fallback(self):
        source = (ROOT / 'Application/DeviceLayer/Sensor/imu_sensor.c').read_text(encoding='utf-8-sig')
        start = source.index('    float real_gyrox = gyrox_;')
        end = source.index('    /* 清零离线计数器', start)
        for mode in (0,1):
            run_c(f'#define IMU_USE_EKF {mode}\n' + r'''
#include <math.h>
#include <assert.h>
struct {float GyroBias[3];} QEKF_INS={{0.1f,-0.2f,0}};
struct rates {float rate_pitch,rate_roll,rate_yaw,ave_rate_pitch,ave_rate_roll,ave_rate_yaw;};
struct info {struct rates base_info;};
static int imu_pitch_dif_speed_ave_filter,imu_roll_dif_speed_ave_filter,imu_yaw_dif_speed_ave_filter;
float ave_fil_update(int *f,float x,int n){return x;}
int main(void){
    struct info storage={0},*imu_info=&storage;
    float gyrox_=0.1f,gyroy_=-0.2f,gyroz_=0.3f;
''' + source[start:end] + r'''
#if IMU_USE_EKF
    assert(fabsf(storage.base_info.rate_pitch)<1e-5f);
    assert(fabsf(storage.base_info.rate_roll)<1e-5f);
#else
    assert(fabsf(storage.base_info.rate_pitch+0.2f/0.0174532925f)<1e-5f);
    assert(fabsf(storage.base_info.rate_roll-0.1f/0.0174532925f)<1e-5f);
#endif
    assert(fabsf(storage.base_info.rate_yaw-0.3f/0.0174532925f)<1e-5f);
    return 0;
}
''')

    def test_installed_gravity_matches_predicted_gravity(self):
        source = (ROOT / 'Application/DeviceLayer/Sensor/imu_sensor.c').read_text(encoding='utf-8-sig')
        start = source.index('static void InitQuaternion(float *init_q4)\n{')
        end = source.index('\n}\n', start) + 3
        initializer = source[start:end]
        harness = r'''
#include <stdint.h>
#include <math.h>
#include <stdio.h>
static float sample[3], temp;
static void BMI088_read(float *g,float *a,float *t) {
    for(int i=0;i<3;i++){ g[i]=0; a[i]=sample[i]; } *t=40;
}
static void delay_ms(int ms) {(void)ms;}
static void arm_sqrt_f32(float x,float *y){*y=sqrtf(x);}
static void arm_atan2_f32(float y,float x,float *r){*r=atan2f(y,x);}
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
static void Vector_Transform(float gx,float gy,float gz,float ax,float ay,float az,
    float *ox,float *oy,float *oz,float *x,float *y,float *z){
    *ox=gx; *oy=-gy; *oz=-gz; *x=ax; *y=-ay; *z=-az;
}
'''
        main = r'''
int main(void) {
    const float poses[][2]={{0,0},{0.35f,0},{0,-0.4f},{0.35f,-0.4f}};
    for(int i=0;i<4;i++) {
        float p=poses[i][0],r=poses[i][1],q[4];
        float expected[3]={-sinf(p),sinf(r)*cosf(p),cosf(r)*cosf(p)};
        sample[0]=expected[0]*9.8f;
        sample[1]=-expected[1]*9.8f; sample[2]=-expected[2]*9.8f;
        InitQuaternion(q);
        float actual[3]={2*(q[1]*q[3]-q[0]*q[2]),
            2*(q[0]*q[1]+q[2]*q[3]),
            q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]};
        for(int j=0;j<3;j++) if(fabsf(actual[j]-expected[j])>1e-5f){
            fprintf(stderr,"pose %d axis %d: %f != %f\n",i,j,actual[j],expected[j]);return 1;
        }
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            c = pathlib.Path(directory) / 'init.c'
            exe = pathlib.Path(directory) / 'init.exe'
            c.write_text(harness + initializer + main, encoding='utf-8')
            subprocess.run(['gcc', '-std=c99', str(c), '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
