#include "chassis_kinematics.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define EPSILON 0.001f

static unsigned int tests_run;

static void expect_wheels(const char *name,
                          float vx, float vy, float wz, float max_rpm,
                          const float expected[CHASSIS_WHEEL_COUNT])
{
    float actual[CHASSIS_WHEEL_COUNT] = {0.0f};

    Chassis_Mecanum_Calculate(vx, vy, wz, max_rpm, actual);
    tests_run++;

    for (unsigned int i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        if (fabsf(actual[i] - expected[i]) >= EPSILON) {
            fprintf(stderr, "%s wheel %u: expected %.3f, got %.3f\n",
                    name, i, expected[i], actual[i]);
            exit(EXIT_FAILURE);
        }
    }
}

int main(void)
{
    const float zero[] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float forward[] = {6000.0f, 6000.0f, 6000.0f, 6000.0f};
    const float lateral[] = {-6000.0f, 6000.0f, 6000.0f, -6000.0f};
    const float rotate[] = {-6000.0f, 6000.0f, -6000.0f, 6000.0f};
    const float saturated[] = {-2000.0f, 6000.0f, 2000.0f, 2000.0f};

    expect_wheels("zero", 0.0f, 0.0f, 0.0f, 6000.0f, zero);
    expect_wheels("forward", 1.0f, 0.0f, 0.0f, 6000.0f, forward);
    expect_wheels("lateral", 0.0f, 1.0f, 0.0f, 6000.0f, lateral);
    expect_wheels("rotate", 0.0f, 0.0f, 1.0f, 6000.0f, rotate);
    expect_wheels("saturated", 1.0f, 1.0f, 1.0f, 6000.0f, saturated);

    printf("%u tests passed\n", tests_run);
    return EXIT_SUCCESS;
}
