#include "chassis.h"
#include "chassis_motor.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EPSILON 0.001f

Motor_RM_t Chassis_Motor[4];
Motor_RM_Group_t Chassis_Motor_Group;

static Motor_RM_Ctrl_Info_t motor_ctrl[4];
static Motor_RM_Tx_Info_t motor_tx[4];
static pid_ctrl_t speed_pid[4];
static unsigned int speed_calls[4];
static unsigned int pid_clear_calls;
static unsigned int group_send_calls;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

static void expect_near(float actual, float expected, const char *message)
{
    if (fabsf(actual - expected) >= EPSILON) {
        fprintf(stderr, "%s: expected %.3f, got %.3f\n",
                message, expected, actual);
        exit(EXIT_FAILURE);
    }
}

void pid_clear(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid_clear_calls++;
}

static void fake_set_speed(Motor_RM_t *motor)
{
    const unsigned int index = (unsigned int)(motor - Chassis_Motor);
    speed_calls[index]++;
    motor->tx_info->torque = motor->ctrl->speed_ctrl->target;
}

static void fake_group_send(Motor_RM_Group_t *group)
{
    (void)group;
    group_send_calls++;
}

static void reset_fixture(void)
{
    memset(Chassis_Motor, 0, sizeof(Chassis_Motor));
    memset(&Chassis_Motor_Group, 0, sizeof(Chassis_Motor_Group));
    memset(motor_ctrl, 0, sizeof(motor_ctrl));
    memset(motor_tx, 0, sizeof(motor_tx));
    memset(speed_pid, 0, sizeof(speed_pid));
    memset(speed_calls, 0, sizeof(speed_calls));
    pid_clear_calls = 0;
    group_send_calls = 0;

    for (unsigned int i = 0; i < 4; ++i) {
        motor_ctrl[i].speed_ctrl = &speed_pid[i];
        Chassis_Motor[i].ctrl = &motor_ctrl[i];
        Chassis_Motor[i].tx_info = &motor_tx[i];
        Chassis_Motor[i].single_set_speed = fake_set_speed;
        Chassis_Motor_Group.motor[i] = &Chassis_Motor[i];
    }
    Chassis_Motor_Group.group_set_torque = fake_group_send;
}

static void test_forward_applies_motor_directions(void)
{
    const float expected[4] = {6000.0f, -6000.0f, 6000.0f, -6000.0f};
    reset_fixture();

    Chassis_Update(660, 0, 0, false);

    for (unsigned int i = 0; i < 4; ++i) {
        expect_near(speed_pid[i].target, expected[i], "forward target");
        if (speed_calls[i] != 1) fail("speed controller must run once");
    }
    if (group_send_calls != 1) fail("normal update must send exactly once");
}

static void test_small_input_has_no_deadzone(void)
{
    reset_fixture();
    Chassis_Update(1, 0, 0, false);

    if (fabsf(speed_pid[0].target) < EPSILON) {
        fail("one-count input was incorrectly removed by a deadzone");
    }
}

static void test_failsafe_clears_state_and_sends_zero_once(void)
{
    reset_fixture();
    for (unsigned int i = 0; i < 4; ++i) {
        speed_pid[i].target = 1.0f;
        speed_pid[i].measure = 2.0f;
        speed_pid[i].err = 3.0f;
        speed_pid[i].last_err = 4.0f;
        speed_pid[i].pout = 5.0f;
        speed_pid[i].iout = 6.0f;
        speed_pid[i].dout = 7.0f;
        speed_pid[i].last_dout = 8.0f;
        speed_pid[i].out = 9.0f;
        speed_pid[i].integral = 10.0f;
        motor_tx[i].torque = 11.0f;
        motor_tx[i].torque_current = 12.0f;
        motor_tx[i].torque_current_raw = 13;
    }

    Chassis_Update(660, 660, 660, true);

    for (unsigned int i = 0; i < 4; ++i) {
        const float *fields[] = {
            &speed_pid[i].target, &speed_pid[i].measure, &speed_pid[i].err,
            &speed_pid[i].last_err, &speed_pid[i].pout, &speed_pid[i].iout,
            &speed_pid[i].dout, &speed_pid[i].last_dout, &speed_pid[i].out,
            &speed_pid[i].integral, &motor_tx[i].torque,
            &motor_tx[i].torque_current,
        };
        for (unsigned int field = 0; field < sizeof(fields) / sizeof(fields[0]); ++field) {
            expect_near(*fields[field], 0.0f, "failsafe state");
        }
        if (motor_tx[i].torque_current_raw != 0) fail("raw current was not cleared");
        if (speed_calls[i] != 0) fail("failsafe must not run speed PID");
    }
    if (pid_clear_calls != 4) fail("failsafe must clear all four PID integrals");
    if (group_send_calls != 1) fail("failsafe must send exactly one zero frame");
}

static void test_control_recovers_after_failsafe(void)
{
    reset_fixture();
    Chassis_Update(0, 0, 0, true);
    group_send_calls = 0;

    Chassis_Update(330, 0, 0, false);

    expect_near(speed_pid[0].target, 3000.0f, "recovery target");
    if (speed_calls[0] != 1 || group_send_calls != 1) {
        fail("normal control did not resume after failsafe");
    }
}

int main(void)
{
    test_forward_applies_motor_directions();
    test_small_input_has_no_deadzone();
    test_failsafe_clears_state_and_sends_zero_once();
    test_control_recovers_after_failsafe();
    puts("4 chassis control tests passed");
    return EXIT_SUCCESS;
}
