"""Run mouse target generation, chassis routing and wheel targets as real C."""
import unittest
import test_keyboard_gyro as fixture


class KeyboardFeedforwardTest(unittest.TestCase):
    run_case = fixture.KeyboardGyroTest.run_case

    def run_mouse_case(self, body, preamble=''):
        self.run_case(r'''
    hero_car.input_src=INPUT_SRC_KEY;hero_car.mode=CAR_MODE_GYRO;
    hero_gimbal.mode=GIMBAL_MODE_GYRO;
    Gimbal_Key_Yaw_Feedforward_Enable(&hero_gimbal,CAR_MODE_GYRO);
    rc.mouse_x=20;rc.mouse_vx=20;rc.frame_seq=1;
    tick=114;rc.frame_received=true;rc.frame_update_ms=114;rc.frame_interval_ms=14;
    int16_t f,l,r;
    Gimbal_Key_Yaw_Update(&hero_gimbal);
    Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);
    Chassis_Update(f,l,r,0,ALIGN_FRONT,CAR_MODE_GYRO,false);
''' + body, preamble)

    def test_mouse_starts_chassis_turn_before_mechanical_angle_error(self):
        self.run_mouse_case(r'''
    near(hero_gimbal.yaw.target_imu_angle,-.02f);
    CHECK(wheel_pid[0].target*CHASSIS_FL_DIRECTION>0 && wheel_pid[1].target*CHASSIS_FR_DIRECTION<0);
    CHECK(wheel_pid[2].target*CHASSIS_BL_DIRECTION>0 && wheel_pid[3].target*CHASSIS_BR_DIRECTION<0);
''')

    def test_same_dbus_frame_is_integrated_once_and_speed_is_held(self):
        self.run_mouse_case(r'''
    CHECK(r==190);near(hero_gimbal.yaw.key_yaw_speed_rads,-.02f/.014f);
    for(int i=0;i<6;i++){
        tick+=2;Gimbal_Key_Yaw_Update(&hero_gimbal);
        Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);
        CHECK(r==190);near(hero_gimbal.yaw.target_imu_angle,-.02f);
        CHECK(hero_gimbal.yaw.key_yaw_ff_valid);
    }
''')

    def test_equal_mouse_speed_at_different_frame_rates_has_equal_feedforward(self):
        self.run_mouse_case(r'''
    int16_t first=r;float first_speed=hero_gimbal.yaw.key_yaw_speed_rads;
    rc.frame_seq++;tick+=28;rc.frame_update_ms=tick;rc.frame_interval_ms=28;
    rc.mouse_x=40;rc.mouse_vx=40;Gimbal_Key_Yaw_Update(&hero_gimbal);
    Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);
    CHECK(r==first);near(hero_gimbal.yaw.key_yaw_speed_rads,first_speed);
    near(hero_gimbal.yaw.target_imu_angle,-.06f);
''')

    def test_zero_raw_mouse_stops_feedforward_even_with_existing_average_tail(self):
        self.run_mouse_case(r'''
    CHECK(chassis_yaw_ff_filtered<0);
    rc.frame_seq++;tick+=14;rc.frame_update_ms=tick;rc.mouse_vx=0;rc.mouse_x=10;
    Gimbal_Key_Yaw_Update(&hero_gimbal);Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);
    CHECK(r==0);CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);
    near(hero_gimbal.yaw.target_imu_angle,-.03f);
    Chassis_Update(f,l,r,0,ALIGN_FRONT,CAR_MODE_GYRO,false);
    near(chassis_yaw_ff_filtered,0);for(int i=0;i<4;i++)near(wheel_pid[i].target,0);
''')

    def test_fifty_ms_without_new_frame_expires_feedforward(self):
        self.run_mouse_case(r'''
    tick=163;Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(hero_gimbal.yaw.key_yaw_ff_valid);
    tick=164;Gimbal_Key_Yaw_Update(&hero_gimbal);Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);
    CHECK(r==0);CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);near(hero_gimbal.yaw.key_yaw_speed_rads,0);
    near(hero_gimbal.yaw.target_imu_angle,-.02f);
''')

    def test_invalid_timing_requires_new_continuous_frame(self):
        self.run_mouse_case(r'''
    uint32_t gaps[]={0,51,1000};
    for(unsigned i=0;i<3;i++){
        rc.frame_seq++;rc.frame_interval_ms=gaps[i];rc.frame_update_ms=tick;
        Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);
        CHECK(hero_gimbal.yaw.key_yaw_ff_channel==0);
    }
    rc.frame_seq++;rc.frame_interval_ms=14;rc.frame_received=false;
    Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);
    rc.frame_received=true;rc.frame_seq++;
    Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(hero_gimbal.yaw.key_yaw_ff_valid);
''')

    def test_gain_and_extreme_inputs_are_bounded_without_changing_aim_sensitivity(self):
        self.run_mouse_case(r'''
    hero_gimbal.yaw.key_yaw_ff_gain=.5f;rc.frame_seq++;tick+=14;rc.frame_update_ms=tick;
    Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(hero_gimbal.yaw.key_yaw_ff_channel==95);
    near(hero_gimbal.yaw.target_imu_angle,-.04f);
    rc.frame_seq++;rc.mouse_x=32767;rc.mouse_vx=32767;rc.frame_interval_ms=1;
    Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(hero_gimbal.yaw.key_yaw_ff_channel==660);
    rc.frame_seq++;rc.mouse_x=-32768;rc.mouse_vx=-32768;
    Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(hero_gimbal.yaw.key_yaw_ff_channel==-660);
''')

    def test_invalid_configuration_and_zero_gain_stop_a_held_command(self):
        self.run_mouse_case(r'''
    const float gains[]={0,-1,NAN,INFINITY};
    for(unsigned i=0;i<4;i++){
        hero_gimbal.yaw.key_yaw_ff_gain=gains[i];Gimbal_Key_Yaw_Update(&hero_gimbal);
        CHECK(hero_gimbal.yaw.key_yaw_ff_channel==0);CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);
    }
    hero_gimbal.yaw.key_yaw_ff_gain=1;
    const float sensitivities[]={0,-1,NAN,INFINITY};
    for(unsigned i=0;i<4;i++){
        hero_gimbal.yaw.gyro_sensitivity=sensitivities[i];rc.frame_seq++;
        Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);
    }
''')

    def test_nonfinite_mouse_does_not_poison_target_or_feedforward(self):
        self.run_mouse_case(r'''
    const float values[]={NAN,INFINITY,-INFINITY,FLT_MAX};
    for(unsigned i=0;i<4;i++){
        rc.frame_seq++;rc.mouse_x=values[i];Gimbal_Key_Yaw_Update(&hero_gimbal);
        CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);near(hero_gimbal.yaw.target_imu_angle,-.02f);
    }
''')

    def test_switching_inputs_clears_previous_lowpass_and_waits_for_new_mouse_frame(self):
        self.run_mouse_case(r'''
    hero_car.input_src=INPUT_SRC_RC;rc.ch0=-660;
    Gimbal_Key_Yaw_Feedforward_Enable(&hero_gimbal,CAR_MODE_GYRO);
    Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);CHECK(r==-660);
    Chassis_Update(f,l,r,0,ALIGN_FRONT,CAR_MODE_GYRO,false);
    near(chassis_yaw_ff_filtered,CHASSIS_YAW_FF_LPF_ALPHA);
    hero_car.input_src=INPUT_SRC_KEY;Gimbal_Key_Yaw_Feedforward_Enable(&hero_gimbal,CAR_MODE_GYRO);
    Gimbal_Key_Yaw_Update(&hero_gimbal);Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);CHECK(r==0);
    Chassis_Update(f,l,r,0,ALIGN_FRONT,CAR_MODE_GYRO,false);near(chassis_yaw_ff_filtered,0);
    rc.frame_seq++;tick+=14;rc.frame_update_ms=tick;Gimbal_Key_Yaw_Update(&hero_gimbal);
    Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);CHECK(r==190);
''')

    def test_spin_mech_ctrl_and_device_failures_do_not_use_mouse_feedforward(self):
        self.run_mouse_case(r'''
    hero_car.mode=CAR_MODE_SPIN;Gimbal_Key_Yaw_Feedforward_Enable(&hero_gimbal,CAR_MODE_SPIN);
    Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);CHECK(r==0);
    Chassis_Update(f,l,r,0,ALIGN_FRONT,CAR_MODE_SPIN,false);near(chassis_yaw_ff_filtered,0);
    near(wheel_pid[0].target,-CHASSIS_SPIN_NORMALIZED*CHASSIS_MAX_MOTOR_RPM*CHASSIS_FL_DIRECTION);
    hero_car.mode=CAR_MODE_MECH;hero_gimbal.mode=GIMBAL_MODE_MECH;
    Gimbal_Key_Yaw_Feedforward_Enable(&hero_gimbal,CAR_MODE_MECH);rc.E.cnt=KEY_Q_CNT_MAX;
    Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);CHECK(r==660);
    hero_car.mode=CAR_MODE_GYRO;hero_gimbal.mode=GIMBAL_MODE_GYRO;
    rc.Ctrl.value=1;ctrl_val_snap=1;Gimbal_Key_Yaw_Feedforward_Enable(&hero_gimbal,CAR_MODE_GYRO);
    Car_Chassis_Channel_Get(&hero_car,&f,&l,&r);CHECK(f==0 && l==0 && r==0);
    rc.Ctrl.value=0;ctrl_val_snap=0;
    for(int fault=0;fault<3;fault++){
        rc_sensor.work_state=DEV_ONLINE;rc_sensor.errno=NONE_ERR;board_rx_info.is_online=true;
        if(fault==0)rc_sensor.work_state=DEV_OFFLINE;
        if(fault==1)rc_sensor.errno=DEV_DATA_ERR;
        if(fault==2)board_rx_info.is_online=false;
        Gimbal_Key_Yaw_Feedforward_Enable(&hero_gimbal,CAR_MODE_GYRO);
        CHECK(!hero_gimbal.yaw.key_yaw_ff_active);CHECK(hero_gimbal.yaw.key_yaw_ff_channel==0);
    }
    Chassis_Update(0,0,660,0,ALIGN_FRONT,CAR_MODE_GYRO,true);
    near(chassis_yaw_ff_filtered,0);for(int i=0;i<4;i++)near(wheel_pid[i].target,0);
''')

    def test_sequence_and_timestamp_wrap_are_valid(self):
        self.run_mouse_case(r'''
    rc.frame_seq=65535;tick=UINT32_MAX-5;rc.frame_update_ms=tick;
    Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(hero_gimbal.yaw.key_yaw_ff_valid);
    rc.frame_seq=0;tick=8;rc.frame_update_ms=8;Gimbal_Key_Yaw_Update(&hero_gimbal);
    CHECK(hero_gimbal.yaw.key_yaw_ff_valid);CHECK(hero_gimbal.yaw.key_yaw_ff_channel==190);
    tick=57;Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(hero_gimbal.yaw.key_yaw_ff_valid);
    tick=58;Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(!hero_gimbal.yaw.key_yaw_ff_valid);
''')

    def test_mouse_and_time_are_one_snapshot_and_outer_irq_state_is_preserved(self):
        self.run_mouse_case(r'''
    rc.frame_seq++;rc.mouse_x=20;rc.mouse_vx=20;rc.frame_interval_ms=14;
    after_irq_restore=overwrite_mouse_on_restore;irq_mask=0;
    Gimbal_Key_Yaw_Update(&hero_gimbal);
    CHECK(irq_mask==0);near(hero_gimbal.yaw.target_imu_angle,-.04f);
    near(hero_gimbal.yaw.key_yaw_speed_rads,-.02f/.014f);
    CHECK(hero_gimbal.yaw.key_yaw_ff_channel==190);
    irq_mask=1;Gimbal_Key_Yaw_Update(&hero_gimbal);CHECK(irq_mask==1);
''', r'''
static void overwrite_mouse_on_restore(void){
    after_irq_restore=NULL;rc.frame_seq++;rc.mouse_x=999;rc.mouse_vx=999;rc.frame_interval_ms=28;
}
''')


if __name__ == '__main__':
    unittest.main()
