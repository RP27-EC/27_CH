"""Verify Yaw feedback selection and stale-IMU protection on the real controller."""
import unittest

import test_yaw_derivative as yaw_fixture


class YawSpeedFeedbackTest(unittest.TestCase):
    run_case = yaw_fixture.YawDerivativeTest.run_case

    def test_mechanical_uses_encoder_angle_and_imu_speed(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    g.yaw.mec_angle_pid.kp=2;g.yaw.mec_angle_pid.ki=0;g.yaw.mec_angle_pid.kd=0;
    g.yaw.mec_speed_pid.kp=100;g.yaw.mec_speed_pid.ki=0;g.yaw.mec_speed_pid.kd=0;
    g.yaw.target_mec_angle=.5f;g.yaw.feedback_mec_angle=.25f;
    g.yaw.imu_angle=-1;g.yaw.imu_speed=.2f;g.yaw.feedback_mec_speed=-3;
    Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_speed_pid.target,.5f);
    near(g.yaw.mec_speed_pid.measure,.2f);CHECK(sent_current==30);
    g.yaw.feedback_mec_speed=4;Gimbal_Yaw_Pid_Cal(&g);
    CHECK(sent_current==30);
''')

    def test_mechanical_offline_stops_and_clears_both_pid_sets(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_MECH;g.yaw.pid_mode=GIMBAL_PID_MEC;
    g.yaw.imu_speed=1;g.yaw.feedback_mec_speed=1;
    Gimbal_Yaw_Pid_Cal(&g);CHECK(sent_current!=0);
    board_rx_info.is_online=false;Gimbal_Yaw_Pid_Cal(&g);
    CHECK(sent_current==0 && kt_motor[0].base_info.motor_out==0);
    near(g.yaw.mec_speed_pid.out,0);near(g.yaw.gyro_speed_pid.out,0);
    CHECK(!g.yaw.speed_d_ready);
    board_rx_info.is_online=true;g.yaw.imu_speed=2;
    g.yaw.mec_speed_pid.kd=20;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_speed_pid.measure,2);near(g.yaw.mec_speed_pid.dout,0);
''')

    def test_init_keeps_motor_speed_without_imu(self):
        self.run_case(r'''
    g.mode=GIMBAL_MODE_INIT;g.yaw.pid_mode=GIMBAL_PID_MEC;
    g.yaw.mec_angle_pid.kp=0;g.yaw.mec_angle_pid.ki=0;g.yaw.mec_angle_pid.kd=0;
    g.yaw.mec_speed_pid.kp=100;g.yaw.mec_speed_pid.ki=0;g.yaw.mec_speed_pid.kd=0;
    g.yaw.feedback_mec_speed=.5f;g.yaw.imu_speed=-3;
    board_rx_info.is_online=false;Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.mec_speed_pid.measure,.5f);CHECK(sent_current==-50);
''')

    def test_gyro_keeps_imu_angle_and_speed(self):
        self.run_case(r'''
    g.yaw.gyro_angle_pid.kp=2;g.yaw.gyro_angle_pid.ki=0;g.yaw.gyro_angle_pid.kd=0;
    g.yaw.gyro_speed_pid.kp=100;g.yaw.gyro_speed_pid.ki=0;g.yaw.gyro_speed_pid.kd=0;
    g.yaw.target_imu_angle=.5f;g.yaw.imu_angle=.25f;g.yaw.imu_speed=.2f;
    g.yaw.feedback_mec_angle=-2;g.yaw.feedback_mec_speed=-3;
    Gimbal_Yaw_Pid_Cal(&g);
    near(g.yaw.gyro_speed_pid.target,.5f);
    near(g.yaw.gyro_speed_pid.measure,.2f);CHECK(sent_current==30);
''')


if __name__ == '__main__':
    unittest.main()
