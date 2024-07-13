#ifndef _PID_PARAMETER_H
#define _PID_PARAMETER_H

#include "Struct_typedef.h"

#define IMUTEMP_MAX_OUT  													    500
#define IMUTEMP_MIN_OUT   													  0
#define IMUTEMP_MAX_IOUT   													  100
#define IMUTEMP_MIN_IOUT     													0

#define FOLLOW_MAX_OUT																600
#define FOLLOW_MAX_IOUT																100

#define M3508_MAX_OUT																	16383
#define M3508_MAX_IOUT																5000

#define LEG_F_MAX																			480
#define LEG_F_MIN																			-120
#define LEG_I_MAX																			120
#define LEG_I_MIN																			0

#define BALANCE_KP																		100.0f
#define BALANCE_KI																		0.0f
#define BALANCE_KD																		0.1f



													
#define IMUTEMP_DES    															  40.0f

#define IMUTEMP_KP    															  100.f
#define IMUTEMP_KI    															  50.f
#define IMUTEMP_KD   			 												    10.f

fp32 IMUtemp_PID[3]={IMUTEMP_KP,IMUTEMP_KI,IMUTEMP_KD};


#define NOFORCE_KP																			0.0f
#define NOFORCE_KI																			0.0f
#define NOFORCE_KD																			0.0f
fp32 Noforce_PID[3]={NOFORCE_KP,NOFORCE_KI,NOFORCE_KD};

#define DASH_FOLLOW_KP																	10.0f
#define DASH_FOLLOW_KI																	0.0f
#define DASH_FOLLOW_KD																	100.0f
fp32 Dash_FOLLOW_PID[3]={DASH_FOLLOW_KP,DASH_FOLLOW_KI,DASH_FOLLOW_KD};


#define FOLLOW_KP																				12.0f
#define FOLLOW_KI																				0.0f
#define FOLLOW_KD																				100.0f
fp32 Follow_PID[3]={FOLLOW_KP,FOLLOW_KI,FOLLOW_KD};





#define TURN_KP_LEFT																		3000.0f
#define TURN_KI_LEFT																		0.0f
#define TURN_KD_LEFT																		10000.0f
fp32 Turn_left_PID[3]={TURN_KP_LEFT,TURN_KI_LEFT,TURN_KD_LEFT};

#define TURN_KP_RIGHT																		3000.0f
#define TURN_KI_RIGHT																		0.0f
#define TURN_KD_RIGHT																		10000.0f
fp32 Turn_right_PID[3]={TURN_KP_RIGHT,TURN_KI_RIGHT,TURN_KD_RIGHT};



#define STOP_KP																					8000.0f
#define STOP_KI																					0.0f
#define STOP_KD																					0.0f
fp32 Stop_PID[3]={STOP_KP,STOP_KI,STOP_KD};


#define LEG_KP																					2000.0f
#define LEG_KI																					0.1f
#define LEG_KD																					20.0f
fp32 Leg_PID[3]={LEG_KP,LEG_KI,LEG_KD};
	
	
#define ROLL_KP																					0.1f
#define ROLL_KI																					0.0001f
#define ROLL_KD																					0.00f
fp32 Roll_PID[3]={ROLL_KP,ROLL_KI,ROLL_KD};


#define ANGLE_KP																				50000.0f
#define ANGLE_KI																				0.0f
#define ANGLE_KD																				100.0f
fp32 Angle_PID[3]={ANGLE_KP,ANGLE_KI,ANGLE_KD};

#define SPEED_KP																				0.02f
#define SPEED_KI																				0.0002f
#define SPEED_KD																				0.0f
fp32 Speed_PID[3]={SPEED_KP,SPEED_KI,SPEED_KD};

#endif

