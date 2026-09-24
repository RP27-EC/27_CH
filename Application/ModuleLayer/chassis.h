#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_RC_MAX_VALUE  660.0f
#define CHASSIS_MAX_MOTOR_RPM 6000.0f

void Chassis_Init(void);
void Chassis_Update(int16_t forward, int16_t lateral, int16_t rotate,
                    bool failsafe);
void Chassis_Stop(void);

#endif
