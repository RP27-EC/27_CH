#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "device.h"

/* 任务入口函数声明 */
void StartControlTask(void const * argument);

#endif /* __CONTROL_TASK_H */
