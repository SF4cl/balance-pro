#ifndef _SETTING_H
#define _SETTING_H

#include "Struct_typedef.h"

#define M3508_Feedback_Frequency		1000.0f
#define A1_Feedback_Frequency				1000.0f
#define IMUTask_frequency						1000.0f
#define ChassisTask_frequency				1000.0f

#define Calf											0.24f				//小腿/m
#define Thigh											0.12f				//大腿/m
#define Innominate 								0.15f				//间距/m

#define Wheel_weight							1.14f				//轮毂重量/kg
#define Wheel_diameter						0.1f				//轮径/m
#define Reduction_ratio						15.76f			//减速比·






#define FILTER_PARAMETER_JOINT_ANGLE			0.00f
#define FILTER_PARAMETER_JOINT_SPEED			0.00f
#define FILTER_PARAMETER_JOINT_ALPHA			0.000f
#define FILTER_PARAMETER_JOINT_DALPHA			0.000f
#define FILTER_PARAMETER_JOINT_LENGTH			0.000f
#define FILTER_PARAMETER_JOINT_DLENGTH		0.000f
#define FILTER_PARAMETER_JOINT_FORCE			0.003f
#define FILTER_PARAMETER_ACCEL						0.005f
#define FILTER_PARAMETER_TARGET_X					0.15f
#define FILTER_PARAMETER_TARGET_DX				0.15f




#define FOLLOW_SPEED_MAX									1.7f
#define DASH_SPEED_MAX										2.6f
#define STOP_SPEED_MAX										1.5f




#define PARAMETER_FILE "Setting.h"





#define YawMotorId                                   	0x205

#define FEET_MOTOR_TRANSMIT_ID         			  		  	0x200

#define FEET_MOTOR1_RECEIVE_ID         	  						0x202
#define FEET_MOTOR2_RECEIVE_ID         	 			 		    0x201


#define IMU_GYRO_YAW_BIAS    													-0.00465f

#define FOLLOW_ANGLE              										-161.0f
#define ROTING_SPEED																	-8.0f

#define TORQUE_K																			2730.0f*1.22f
#define TORQUE_W																			0.754f



#define CMSBufferPowerSendID 0x2E
#define CMSDateSendID 0x2F
#define CMSRecceiveID 0x30//0x211-old




#define JOINT_RUN_MODE 																10
#define JOINT_RESET_MODE															0
#define JOINT_KP 																			0.1
#define JOINT_KW 																			0.01
#define JOINT_W 																			0
#define JOINT_T 																			0
#define JOINT_RIGHT1_ID 															0
#define JOINT_RIGHT2_ID 															1
#define JOINT_LEFT1_ID 																0
#define JOINT_LEFT2_ID 																1



#endif

