#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "cmsis_os.h"
#include "bmi088driver.h"
#include "tim.h"
#include "vofa.h"
#include "Setting.h"
#include "QuaternionEKF.h"
#include "Struct_typedef.h"
#include "Config.h"
#include "pid.h"
#include "MahonyAHRS.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2






#endif
