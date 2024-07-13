#ifndef FEET_MOTOR_H
#define FEET_MOTOR_H

#include "fdcan.h"
#include "Setting.h"
#include "feet_motor.h"
#include "stm32h7xx_it.h"
#include "Struct_typedef.h"
#include "CanPacket.h"
#include "CMS.h"
#include "chassis_task.h"





#define get_feet_motor_measure(ptr, data)                               \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
				(ptr)->last_speed_rpm = (ptr)->speed_rpm;      									\
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperature = (data)[6];                                 \
			  (ptr)->angle = (ptr)->ecd / 8191.0f * 360 - 180;          			\
    }

#define get_yaw_motor_measure(ptr, data)                                \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
				(ptr)->last_speed_rpm = (ptr)->speed_rpm;      									\
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperature = (data)[6];                                 \
			  (ptr)->angle = (ptr)->ecd / 8191.0f * 360 - 180;          			\
				(ptr)->speed = (ptr)->speed_rpm / 60.0f * 3.14159f * 2.0f;			\
    }		
		

extern void FEET_CONTROL(int16_t FEET_MOTOR_LEFT, int16_t FEET_MOTOR_RIGHT);

		
#endif
		