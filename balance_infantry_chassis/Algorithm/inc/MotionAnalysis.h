#ifndef MOTIONANALUSIS_H
#define MOTIONANALUSIS_H

#include "arm_math.h"
#include "Struct_typedef.h"
#include "user_lib.h"
#include "Setting.h"
#include "joint_motor.h"

extern void Joint_init(JOINT_t *joint);
extern void AttitudeCalc(JOINT_t *joint,IMUData_t *imu);
extern uint8_t CommandCalc(JOINT_t *joint,fp32 Force,fp32 Torque);

#endif



