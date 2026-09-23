#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "device.h"
#include "chassis.h"
#include "rc_sensor.h"

/* 导出全局底盘控制对象，供 CAN 接收中断回调函数解析反馈 */
extern Chassis_t hero_chassis;

/* 任务入口函数声明 */
void StartControlTask(void const * argument);

#endif /* __CONTROL_TASK_H */