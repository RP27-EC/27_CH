"""Exercise the real power adapter between real chassis PID calls and CAN send."""
import unittest

import test_keyboard_gyro as fixture


class PowerIntegrationTest(unittest.TestCase):
    run_case = fixture.KeyboardGyroTest.run_case

    def test_all_movement_modes_limit_before_group_send(self):
        self.run_case(r'''
    const CarMode_e modes[]={CAR_MODE_MECH,CAR_MODE_GYRO,CAR_MODE_SPIN};
    for(unsigned m=0;m<3;m++){
        Chassis_Power_Init();
        for(int i=0;i<4;i++)Chassis_Motor[i].rx_info->encoder_speed=(i%2)?-3000:3000;
        Chassis_Update(660,0,660,.2f,ALIGN_FRONT,modes[m],false);
        CHECK(chassis_power.limiting_active);
        CHECK(chassis_power.predicted_after_w<=chassis_power.target_power_w+.001f);
        for(int i=0;i<4;i++)near(sent_current[i],chassis_power.current_after[i]);
    }
''')

    def test_adapter_consumes_separate_freshness_and_failsafe_still_sends_zero(self):
        self.run_case(r'''
    tick=100;My_Judge.status->limit_received=true;
    My_Judge.status->limit_rx_count=1;My_Judge.status->limit_update_ms=100;
    My_Judge.org_info->game_robot_status.chassis_power_limit=60;
    Chassis_Update(660,0,0,0,ALIGN_FRONT,CAR_MODE_MECH,false);
    near(chassis_power.base_budget_w,55);CHECK(chassis_power.judge_limit_valid);
    tick=1000;My_Judge.status->buffer_received=true;
    My_Judge.status->buffer_rx_count=1;My_Judge.status->buffer_update_ms=1000;
    My_Judge.org_info->power_heat_data.chassis_power_buffer=40;
    cap.status->power_received=true;cap.status->power_rx_count=1;
    cap.status->power_update_ms=1000;cap.info->chassis_power=50;
    Chassis_Update(660,0,0,0,ALIGN_FRONT,CAR_MODE_MECH,false);
    CHECK(!chassis_power.judge_limit_valid && chassis_power.buffer_valid && chassis_power.cap_valid);
    near(chassis_power.target_power_w,40);
    tick=2000;Chassis_Update(660,0,0,0,ALIGN_FRONT,CAR_MODE_MECH,false);
    near(chassis_power.target_power_w,40);CHECK(chassis_power.current_scale>0);
    Chassis_Update(660,0,0,0,ALIGN_FRONT,CAR_MODE_MECH,true);
    for(int i=0;i<4;i++)near(sent_current[i],0);
    near(chassis_power.current_scale,0);near(chassis_power.target_power_w,40);
''')


if __name__ == '__main__':
    unittest.main()
