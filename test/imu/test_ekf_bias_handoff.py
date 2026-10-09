"""Exercise the actual calibration block and EKF handoff with host sensor state."""
import unittest
from test_ekf_init import ROOT, run_c


class BiasHandoffTest(unittest.TestCase):
    def test_calibration_transition_is_continuous_and_applied_once(self):
        sensor = (ROOT / 'Application/DeviceLayer/Sensor/imu_sensor.c').read_text(encoding='utf-8-sig')
        ekf = (ROOT / 'Application/DeviceLayer/Imu/bmi_EKF.c').read_text(encoding='utf-8-sig')
        signature = 'void IMU_QuaternionEKF_ShiftGyroBias('
        self.assertTrue(signature in ekf, 'EKF needs a state-and-cache bias handoff API')
        start = ekf.index(signature)
        handoff = ekf[start:ekf.index('\n}\n', start) + 3]
        start = sensor.index('    /* ========== 步骤3：')
        calibration = sensor[start:sensor.index('    /* ========== 步骤4：', start)]
        for mode in (0, 1):
            with self.subTest(ekf=mode):
                run_c(f'#define IMU_USE_EKF {mode}\n#define IMU_USE_MAHONY {1-mode}\n' + r'''
#include <stdint.h>
#include <math.h>
#include <assert.h>
#define abs(x) fabsf(x)
#define IMU_DATA_CALI 1
#define IMU_NONE_ERR 0
typedef struct {
    float xhat_data[6],xhatminus_data[6],FilteredValue[6],P_data[36];
} KalmanFilter_t;
struct {
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    float GyroBias[3];
} QEKF_INS;
struct info {struct {float gx_offset,gy_offset,gz_offset;} offset_info;};
struct sensor {struct {int err_code,cali_end;} work_state;struct info *info;};
static unsigned imu_cnt;
static float gyrox,gyroy,gyroz;
''' + handoff + r'''
void step(struct sensor *imu_sen){
    struct info *imu_info=imu_sen->info;
''' + calibration + r'''
}
static void near(float a,float b){assert(fabsf(a-b)<1e-7f);}
int main(void){
    struct info info={0};
    struct sensor sensor={{IMU_DATA_CALI,0},&info};
    KalmanFilter_t *kf=&QEKF_INS.IMU_QuaternionEKF;
    QEKF_INS.Initialized=1;
    kf->xhat_data[0]=1; kf->P_data[28]=0.125f;
    kf->xhat_data[4]=kf->xhatminus_data[4]=kf->FilteredValue[4]=QEKF_INS.GyroBias[0]=0.0018f;
    kf->xhat_data[5]=kf->xhatminus_data[5]=kf->FilteredValue[5]=QEKF_INS.GyroBias[1]=-0.0028f;
    for(int i=0;i<2000;i++){
        gyrox=0.002f;gyroy=-0.003f;gyroz=0.001f;
        step(&sensor);
        assert(sensor.work_state.err_code==IMU_DATA_CALI);
        near(gyrox,0.002f);near(gyroy,-0.003f);
        near(QEKF_INS.GyroBias[0],0.0018f);
    }
    for(int i=0;i<3;i++){
        gyrox=0.002f;gyroy=-0.003f;gyroz=0.001f;
        step(&sensor);
        assert(sensor.work_state.err_code==IMU_NONE_ERR && sensor.work_state.cali_end);
        near(gyrox,0);near(gyroy,0);near(gyroz,0);
#if IMU_USE_EKF
        /* No impulse on the handoff frame, and no repeated shift next frame. */
        near(gyrox-QEKF_INS.GyroBias[0],0.0002f);
        near(gyroy-QEKF_INS.GyroBias[1],-0.0002f);
        near(kf->xhat_data[4],QEKF_INS.GyroBias[0]);
        near(kf->xhatminus_data[4],QEKF_INS.GyroBias[0]);
        near(kf->FilteredValue[4],QEKF_INS.GyroBias[0]);
        near(kf->xhat_data[5],QEKF_INS.GyroBias[1]);
        near(kf->xhatminus_data[5],QEKF_INS.GyroBias[1]);
        near(kf->FilteredValue[5],QEKF_INS.GyroBias[1]);
#else
        near(QEKF_INS.GyroBias[0],0.0018f);
#endif
        near(kf->xhat_data[0],1);near(kf->P_data[28],0.125f);
    }
    /* Existing rejection policy: discarded offsets must never shift state. */
    sensor.work_state.err_code=IMU_DATA_CALI;
    sensor.work_state.cali_end=0;imu_cnt=2000;
    info.offset_info.gx_offset=0.006f;
    info.offset_info.gy_offset=-0.006f;
    info.offset_info.gz_offset=0.006f;
    float old_x=QEKF_INS.GyroBias[0],old_y=QEKF_INS.GyroBias[1];
    gyrox=0.002f;gyroy=-0.003f;gyroz=0.001f;step(&sensor);
    near(QEKF_INS.GyroBias[0],old_x);near(QEKF_INS.GyroBias[1],old_y);
    near(gyrox,0.002f);near(gyroy,-0.003f);near(gyroz,0.001f);
    return 0;
}
''')


if __name__ == '__main__':
    unittest.main()
