#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "KT_motor.h"
#include "HT_motor.h"
#include "DM_motor.h"
#include "motor_def.h"
#include "drv_can.h"



/*电机ID宏定义------------------------------------------------*/
#define ID_GIMB_YAW 	0x142

/* 六摩擦轮 RM3508 反馈 ID（CAN1，反馈报文 ID = 0x200 + 电机ID） */
#define ID_FRIC_F_U		0x201	/* 前层上，0x200 组槽位 0 */
#define ID_FRIC_F_R		0x202	/* 前层右，0x200 组槽位 1 */
#define ID_FRIC_F_L		0x203	/* 前层左，0x200 组槽位 2 */
#define ID_FRIC_B_U		0x205	/* 后层上，0x1FF 组槽位 0 */
#define ID_FRIC_B_R		0x206	/* 后层右，0x1FF 组槽位 1 */
#define ID_FRIC_B_L		0x207	/* 后层左，0x1FF 组槽位 2 */

extern  KT_motor_t kt_motor[1];
extern  Motor_HT_t L_Wheel;
extern  Motor_DM_t Yaw_Motor;

/* 六摩擦轮电机（RM3508，CAN1）：前层 F_* 走 0x200 组，后层 B_* 走 0x1FF 组 */
extern  Motor_RM_t F_U_Fric;
extern  Motor_RM_t F_R_Fric;
extern  Motor_RM_t F_L_Fric;
extern  Motor_RM_t B_U_Fric;
extern  Motor_RM_t B_R_Fric;
extern  Motor_RM_t B_L_Fric;
extern  Motor_RM_Group_t Fric_F_Group;
extern  Motor_RM_Group_t Fric_B_Group;
/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void kt_motor_list_init(void);
void ht_motor_list_init(void);
void dm_motor_list_init(void);

#endif

