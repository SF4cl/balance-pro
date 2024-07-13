#ifndef __ESTIMATOR_TASK_H__
#define __ESTIMATOR_TASK_H__

#include "Struct_typedef.h"
#include "estimator_task.h"
#include "arm_math.h"
#include "kalman filter.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "feet_motor.h"
#include "imu_task.h"
#include "Config.h"
#include "MotionAnalysis.h"

extern void EstimatorTask_Entry(void const * argument);




#endif