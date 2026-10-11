"""Exercise chassis IMU compensation through the actual firmware controller."""
import unittest

import test_yaw_derivative as fixture


class YawChassisFeedforwardTest(unittest.TestCase):
    def run_case(self, body):
        # Exercise the specified gains independently of the robot's tuned default.
        fixture.YawDerivativeTest.run_case(self, 'g.yaw.chassis_fow_rads=1.0f;\n' + body)

    def test_aligned_turret_tracks_chassis_without_angle_error(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    bottom_imu_info.base_info.ave_rate_yaw=90;
    g.yaw.imu_speed=PI/2;
    Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_angle_pid.out,0);
    near(g.yaw.mec_speed_pid.target,PI/2);
    near(g.yaw.mec_speed_pid.err,0);CHECK(sent_current==0);
    bottom_imu_info.base_info.ave_rate_yaw=-90;g.yaw.imu_speed=-PI/2;
    Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_speed_pid.target,-PI/2);CHECK(sent_current==0);
''')

    def test_watch_gain_and_zero_disable_take_effect_on_next_cycle(self):
        self.run_case(r'''
    near(g.yaw.chassis_fow_rads,1);
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    bottom_imu_info.base_info.ave_rate_yaw=90;
    g.yaw.chassis_fow_rads=.5f;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.chassis_speed_rads,PI/2);
    near(g.yaw.chassis_fow_speed,PI/4);
    near(g.yaw.yaw_target_speed,PI/4);CHECK(g.yaw.chassis_fow_valid);
    g.yaw.chassis_fow_rads=0;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.chassis_fow_speed,0);near(g.yaw.mec_speed_pid.target,0);
''')

    def test_compensation_adds_to_angle_correction_and_respects_speed_limit(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    bottom_imu_info.base_info.ave_rate_yaw=90;
    g.yaw.target_mec_angle=.1f;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_speed_pid.target,1.2f+PI/2);
    bottom_imu_info.base_info.ave_rate_yaw=1000;
    Gimbal_Yaw_Pid_Cal(&g);near(g.yaw.mec_speed_pid.target,12);
    g.yaw.target_mec_angle=-.1f;bottom_imu_info.base_info.ave_rate_yaw=-1000;
    Gimbal_Yaw_Pid_Cal(&g);near(g.yaw.mec_speed_pid.target,-12);
''')

    def test_bottom_imu_health_gates_compensation_and_recovers(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    bottom_imu_info.base_info.ave_rate_yaw=90;g.yaw.target_mec_angle=.1f;
    for(int fault=0;fault<4;fault++){
        imu_sensor.info=&bottom_imu_info;
        imu_sensor.work_state.dev_state=DEV_ONLINE;
        imu_sensor.work_state.err_code=IMU_NONE_ERR;
        imu_sensor.work_state.cali_end=1;
        Gimbal_Yaw_Pid_Cal(&g);CHECK(g.yaw.chassis_fow_valid);
        if(fault==0)imu_sensor.info=NULL;
        if(fault==1)imu_sensor.work_state.dev_state=DEV_OFFLINE;
        if(fault==2)imu_sensor.work_state.err_code=IMU_DATA_CALI;
        if(fault==3)imu_sensor.work_state.cali_end=0;
        Gimbal_Yaw_Pid_Cal(&g);
        CHECK(!g.yaw.chassis_fow_valid);
        near(g.yaw.chassis_speed_rads,0);near(g.yaw.chassis_fow_speed,0);
        near(g.yaw.yaw_target_speed,1.2f);CHECK(sent_current>0);
    }
''')

    def test_nonfinite_sample_gain_and_product_never_enter_speed_pid(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    g.yaw.target_mec_angle=.1f;
    const float samples[]={NAN,INFINITY,-INFINITY,90,90,3e38f};
    const float gains[]={1,1,1,NAN,INFINITY,3e38f};
    for(unsigned i=0;i<sizeof(samples)/sizeof(samples[0]);i++){
        bottom_imu_info.base_info.ave_rate_yaw=samples[i];
        g.yaw.chassis_fow_rads=gains[i];Gimbal_Yaw_Pid_Cal(&g);
        CHECK(!g.yaw.chassis_fow_valid);near(g.yaw.chassis_fow_speed,0);
        near(g.yaw.mec_speed_pid.target,1.2f);CHECK(sent_current>0);
    }
''')

    def test_gyro_and_init_do_not_receive_compensation(self):
        self.run_case(r'''
    bottom_imu_info.base_info.ave_rate_yaw=90;
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    Gimbal_Yaw_Pid_Cal(&g);CHECK(g.yaw.chassis_fow_valid);
    g.mode=GIMBAL_MODE_GYRO;g.yaw.pid_mode=GIMBAL_PID_GYRO;
    Gimbal_Yaw_Pid_Cal(&g);near(g.yaw.gyro_speed_pid.target,0);
    CHECK(!g.yaw.chassis_fow_valid);near(g.yaw.chassis_fow_speed,0);
    g.mode=GIMBAL_MODE_INIT;g.yaw.pid_mode=GIMBAL_PID_MEC;
    g.yaw.feedback_mec_speed=.5f;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_speed_pid.target,0);near(g.yaw.mec_speed_pid.measure,.5f);
    CHECK(!g.yaw.chassis_fow_valid);
''')

    def test_sleep_and_upper_imu_timeout_clear_debug_but_preserve_gain(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    bottom_imu_info.base_info.ave_rate_yaw=90;g.yaw.chassis_fow_rads=.5f;
    for(int fault=0;fault<2;fault++){
        g.mode=GIMBAL_MODE_MECH;board_rx_info.is_online=true;
        Gimbal_Yaw_Pid_Cal(&g);CHECK(g.yaw.chassis_fow_valid);
        if(fault==0)g.mode=GIMBAL_MODE_SLEEP;
        else board_rx_info.is_online=false;
        Gimbal_Yaw_Pid_Cal(&g);
        CHECK(sent_current==0);CHECK(!g.yaw.chassis_fow_valid);
        near(g.yaw.chassis_speed_rads,0);near(g.yaw.chassis_fow_speed,0);
        near(g.yaw.yaw_target_speed,0);near(g.yaw.chassis_fow_rads,.5f);
    }
''')


if __name__ == '__main__':
    unittest.main()
