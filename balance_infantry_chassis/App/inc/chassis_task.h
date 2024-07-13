#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "struct_typedef.h"
#include "feet_motor.h"
#include "user_lib.h"
#include "CanPacket.h"
#include "arm_math.h"
#include "imu_task.h"
#include "MotionAnalysis.h"
#include "joint_motor.h"



extern Aim_t Aim;
extern PTZ_t PTZ;
extern UI_t UImessage;

extern void ChassisTask_Entry(void const * argument);

#endif
