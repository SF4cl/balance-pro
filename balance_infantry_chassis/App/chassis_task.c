#include "chassis_task.h"
#include "PID_parameter.h"
/*
TODO:
1、倒地自启PID
2、里程计漂移
3、倾角过大误触离地检测
4、离线检测
5、裁判系统通信

*/
Chassis_t Chassis;
Aim_t Aim;
PTZ_t PTZ;
UI_t UImessage;
Debug_flag_t debug_flag;
first_order_filter_type_t target_x_filter;
first_order_filter_type_t target_dx_filter;

pid_type_def IMUtemp_pid;
pid_type_def follow_pid;
pid_type_def turn_pid_left, turn_pid_right;
pid_type_def leg_pid_left, leg_pid_right;
pid_type_def roll_pid;
pid_type_def stop_pid;
pid_type_def angle_pid_left, angle_pid_right, speed_pid;

uint32_t offtimecount[2] = {0};
uint8_t chassis_state = 0, last_state = 0;
fp32 I_left, I_right, T_left, T_right;
int init_time = 0;

void ChassisInit(void);
void ChassisStateUpdate(void);
void ChassisTargetUpdate(void);
void ChassisCommandUpdate(void);
void TurnPID_Calc(void);
void ForwardPID_Calc(void);
void LegPID_Calc(void);
void LQR_Calc(void);

float Control_Kp[6][2][4] = {
	-246.875846, 289.055008, -166.572495, -1.403837,
	483.473845, -409.075923, 104.392900, 5.982285,
	22.377719, -19.835209, -8.886988, -0.213325,
	34.259803, -35.292013, 13.723643, 0.416141,
	-72.172091, 77.096587, -30.372635, -2.279203,
	0.361333, 31.515981, -31.390860, 11.308734,
	-53.870794, 62.604192, -29.390747, -2.750637,
	-1.463440, 30.447413, -29.845950, 11.424386,
	-1.721240, 71.482050, -69.759237, 25.059609,
	401.040971, -427.886930, 168.411733, 12.331498,
	-17.705305, 24.193019, -13.877302, 4.275510,
	74.481413, -76.189427, 28.867299, 0.025303};

float Calc_Poly(float x, int j, int i)
{
	return x * x * x * Control_Kp[i][j][0] + x * x * Control_Kp[i][j][1] + x * Control_Kp[i][j][2] + Control_Kp[i][j][3];
}

void ChassisTask_Entry(void const *argument)
{
	osDelay(1000);
	ChassisInit();

	while (1)
	{
		ChassisStateUpdate();
		ChassisTargetUpdate();
		ChassisCommandUpdate();

		fp32 pwmout = PID_calc(&IMUtemp_pid, Chassis.IMU.temp, IMUTEMP_DES);
		htim3.Instance->CCR4 = (uint16_t)pwmout;
		osDelay(1);
	}
}

void ChassisInit(void)
{
	debug_flag.mode = 0;
	debug_flag.change_flag = 0;

	Chassis.target_length = 0.15f;

	first_order_filter_init(&target_x_filter, 1.0f / IMUTask_frequency, FILTER_PARAMETER_TARGET_X);
	first_order_filter_init(&target_dx_filter, 1.0f / IMUTask_frequency, FILTER_PARAMETER_TARGET_DX);

	PID_init_extern(&IMUtemp_pid, PID_POSITION, IMUtemp_PID, IMUTEMP_MAX_OUT, IMUTEMP_MAX_IOUT, IMUTEMP_MIN_OUT, IMUTEMP_MIN_IOUT);
	PID_init(&follow_pid, PID_POSITION, Follow_PID, FOLLOW_MAX_OUT, FOLLOW_MAX_IOUT);
	PID_init(&turn_pid_left, PID_POSITION, Turn_left_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	PID_init(&turn_pid_right, PID_POSITION, Turn_right_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	PID_init(&roll_pid, PID_POSITION, Roll_PID, 1, 1);
	PID_init_extern(&leg_pid_left, PID_POSITION, Leg_PID, LEG_F_MAX, LEG_I_MAX, LEG_F_MIN, LEG_I_MIN);
	PID_init_extern(&leg_pid_right, PID_POSITION, Leg_PID, LEG_F_MAX, LEG_I_MAX, LEG_F_MIN, LEG_I_MIN);
	PID_init(&stop_pid, PID_POSITION, Stop_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	PID_init(&angle_pid_left, PID_POSITION, Angle_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	PID_init(&angle_pid_right, PID_POSITION, Angle_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	PID_init(&speed_pid, PID_POSITION, Speed_PID, 0.2, 0.1);
}

void ChassisStateUpdate(void)
{

	if (debug_flag.change_flag == 1)
	{
		Change(Chassis.joint_right.joint_behind.motor_r.angle_filter.out, RIGHT2);
		Change(Chassis.joint_right.joint_forward.motor_r.angle_filter.out, RIGHT1);
		Change(Chassis.joint_left.joint_behind.motor_r.angle_filter.out, LEFT2);
		Change(Chassis.joint_left.joint_forward.motor_r.angle_filter.out, LEFT1);
	}

	fp32 dx = Chassis.Speed_KF.FilteredValue[1];
	fp32 x = target_x_filter.out;
	if (fabs(Chassis.vx) < 0.01f && fabs(Chassis.wz) < 1.0f && abs(PTZ.LRSpeed) == 0 && abs(PTZ.FBSpeed) == 0)
	{
		// if(fabs(x+dx*0.001f)<0.1f)
		x += -dx * 0.001f;
	}
	first_order_filter_cali(&target_x_filter, x);

	switch (PTZ.ChassisStatueRequest)
	{
	case 0x01:
		Chassis.Mode = NOFORCE;
		break;
	case 0x12:
	case 0x92:
		Chassis.Mode = ROTING;
		break;
	case 0x0A:
		Chassis.Mode = FOLLOW;
		break;
	case 0x8A:
		Chassis.Mode = DASH;
		break;
	case 0x06:
	case 0x86:
		Chassis.Mode = STOP;
		break;
	case 0x2A:
	case 0xAA:
		Chassis.Mode = HIGHSPEED;
		break;
	case 0x42:
	case 0xC2:
		Chassis.Mode = JUMP;
		break;
	default:
		break;
	}

	fp32 follow_angle = loop_fp32_constrain(FOLLOW_ANGLE, Chassis.YawMotorMeasure.angle - 180.0f, Chassis.YawMotorMeasure.angle + 180.0f);
	if ((Chassis.Mode == FOLLOW || Chassis.Mode == STOP || Chassis.Mode == DASH) && last_state == ROTING && ((follow_angle - Chassis.YawMotorMeasure.angle) > 0.0f || (follow_angle - Chassis.YawMotorMeasure.angle) < -60.0f))
		Chassis.Mode = ROTING;
	if (Chassis.Mode == ROTING && last_state != ROTING && fabs(dx) > 1.0f)
		Chassis.Mode = last_state;
	if (Chassis.Mode == STOP && last_state != STOP && fabs(dx) > 0.5f)
	{
		if (last_state == DASH)
			Chassis.Mode = last_state, Chassis.vx *= 0.995f;
		else
			Chassis.Mode = last_state, Chassis.vx *= 0.99f;
	}
	if (Chassis.Mode == FOLLOW && last_state == STOP && fabs(dx) > 1.0f)
		Chassis.Mode = last_state;

	if (Chassis.Mode == NOFORCE)
	{
		Joint_motor_set_mode(debug_flag.mode);
		chassis_state = 0;
		last_state = NOFORCE;
	}
	else
	{
		Joint_motor_set_mode(10);
		if (last_state == NOFORCE || last_state == HIGHSPEED && Chassis.Mode != HIGHSPEED)
		{
			chassis_state = 1;
			target_x_filter.out = 0;
			target_dx_filter.out = 0;
			init_time = 0;
		}
		if (chassis_state == 1 && (Chassis.joint_left.length_filter.out < 0.18f && fabs(dx) < 0.1f && Chassis.joint_left.length_filter.out < 0.18f || init_time > 500))
		{
			chassis_state = 2;
			speed_pid.Iout = 0;
			target_x_filter.out = 0;
			target_dx_filter.out = 0;
			init_time = 0;
		}
		if (chassis_state == 2 && (fabs(Chassis.IMU.Angle[1]) < 0.1f && fabs(Chassis.wz) < 0.5f || init_time > 3000))
		{
			chassis_state = 3;

			Change(Chassis.joint_right.joint_behind.motor_r.angle_filter.out, RIGHT2);
			Change(Chassis.joint_right.joint_forward.motor_r.angle_filter.out, RIGHT1);
			Change(Chassis.joint_left.joint_behind.motor_r.angle_filter.out, LEFT2);
			Change(Chassis.joint_left.joint_forward.motor_r.angle_filter.out, LEFT1);
			target_x_filter.out = 0;
			target_dx_filter.out = 0;
			roll_pid.Iout = 0;
			offtimecount[0] = 0;
			offtimecount[1] = 0;
		}
		//				if(chassis_state==3)
		//				{
		//						if(fabs(Chassis.IMU.Angle[1])>25.5f)
		//						{
		//								chassis_state=1;
		//								init_time=0;
		//						}
		//				}
		if (Chassis.Mode == HIGHSPEED)
		{
			chassis_state = 4;
		}
		if (Chassis.Mode == JUMP && last_state != JUMP)
		{
			chassis_state = 5;
			init_time = 0;
		}
		if (Chassis.Mode != JUMP && chassis_state == 5)
		{
			chassis_state = 6;
			init_time = 0;
		}
		if (chassis_state == 6 && ((Chassis.joint_left.length_filter.out > 0.3 && Chassis.joint_right.length_filter.out > 0.3) || init_time > 300))
		{
			chassis_state = 7;
			init_time = 0;
		}
		if (chassis_state == 7 && (init_time > 300))
		{
			chassis_state = 3;
			init_time = 0;
		}
		last_state = Chassis.Mode;
	}

	if (Chassis.joint_left.force_feedback_filter.out < 45.0f && (fabs(Chassis.wz) < 1.5f))
		offtimecount[0]++;
	else
		offtimecount[0] = 0;
	if (offtimecount[0] > 1000)
		offtimecount[0] = 1000;
	if (Chassis.joint_right.force_feedback_filter.out < 45.0f && (fabs(Chassis.wz) < 1.5f))
		offtimecount[1]++;
	else
		offtimecount[1] = 0;
	if (offtimecount[1] > 1000)
		offtimecount[1] = 1000;
}

void ChassisTargetUpdate(void)
{

	float dx = Chassis.Speed_KF.FilteredValue[1];

	if (Chassis.Mode != STOP)
	{
		// first_order_filter_cali(&target_x_filter,target_x_filter.out-PTZ.FBSpeed/32767.0f*2.0f*0.1f);
		if (Chassis.Mode == DASH)
			first_order_filter_cali(&target_dx_filter, -PTZ.FBSpeed / 32767.0f * DASH_SPEED_MAX);
		else
			first_order_filter_cali(&target_dx_filter, -PTZ.FBSpeed / 32767.0f * FOLLOW_SPEED_MAX);
	}
	else
	{
		// first_order_filter_cali(&target_x_filter,target_x_filter.out+PTZ.LRSpeed/32767.0f*2.0f*0.1f);
		first_order_filter_cali(&target_dx_filter, PTZ.LRSpeed / 32767.0f * STOP_SPEED_MAX);
	}
	if (fabs(Chassis.wz) > 2.5f && Chassis.Mode != ROTING && !((PTZ.ChassisStatueRequest & 0x06) != 0 && (Chassis.Mode == FOLLOW || Chassis.Mode == DASH)) && !((PTZ.ChassisStatueRequest & 0x0A) != 0 && (Chassis.Mode == STOP)))
		Chassis.vx = -dx * 0.99f;

	if (PTZ.FBSpeed != 0 || PTZ.LRSpeed != 0)
	{
		if (Chassis.Mode == DASH)
		{
			if (target_dx_filter.out < 0)
				if (Chassis.vx > target_dx_filter.out)
					Chassis.vx += target_dx_filter.out / 1200.0f;
				else
					Chassis.vx -= target_dx_filter.out / 1200.0f;
			else if (Chassis.vx < target_dx_filter.out)
				Chassis.vx += target_dx_filter.out / 1200.0f;
			else
				Chassis.vx -= target_dx_filter.out / 1200.0f;
		}
		else
		{
			if (target_dx_filter.out < 0)
				if (Chassis.vx > target_dx_filter.out)
					Chassis.vx += target_dx_filter.out / 800.0f;
				else
					Chassis.vx *= 0.995f;
			else if (Chassis.vx < target_dx_filter.out)
				Chassis.vx += target_dx_filter.out / 800.0f;
			else
				Chassis.vx *= 0.995f;
		}
	}
	else if (Chassis.Mode == DASH)
		Chassis.vx *= 0.998f;
	else
		Chassis.vx *= 0.993f;

	if (chassis_state == 3 || chassis_state > 4)
	{
		Chassis.target_length += UImessage.mousez * 0.0001f;
		if (Chassis.target_length < 0.08f)
			Chassis.target_length = 0.08f;
		if (Chassis.target_length > 0.32f)
			Chassis.target_length = 0.32f;
		if (chassis_state > 4)
			init_time++;
		if (chassis_state == 3 || chassis_state == 7)
		{
			if ((0x80 & PTZ.ChassisStatueRequest) != Chassis.flag)
			{
				if (0x80 & PTZ.ChassisStatueRequest)
					Chassis.target_length = 0.20f;
				else
					Chassis.target_length = 0.15f;
			}
			Chassis.flag = 0x80 & PTZ.ChassisStatueRequest;
		}
		if (chassis_state == 5)
		{
			if (Chassis.target_length > 0.1f)
				Chassis.target_length -= 0.001f;
		}
		if (chassis_state == 7)
			Chassis.target_length = 0.12f;
	}
}

void ChassisCommandUpdate(void)
{
	TurnPID_Calc();
	ForwardPID_Calc();

	fp32 T_left1, T_left2, T_right1, T_right2;

	if (chassis_state == 0)
	{

		I_right = 0;
		I_left = 0;
		Chassis.joint_left.Torque_forward = 0;
		Chassis.joint_left.Torque_behind = 0;
		Chassis.joint_right.Torque_forward = 0;
		Chassis.joint_right.Torque_behind = 0;
		leg_pid_left.Iout = 30;
		leg_pid_right.Iout = 30;
		roll_pid.Iout = 0;
	}
	if (chassis_state == 1)
	{
		Chassis.joint_left.Torque_forward = -0.3;
		Chassis.joint_left.Torque_behind = 0.3;
		Chassis.joint_right.Torque_forward = 0.3;
		Chassis.joint_right.Torque_behind = -0.3;
		I_left = -stop_pid.out;
		I_right = stop_pid.out;

		init_time++;
	}
	if (chassis_state == 2)
	{
		Chassis.joint_left.Torque_forward = -0.3;
		Chassis.joint_left.Torque_behind = 0.3;
		Chassis.joint_right.Torque_forward = 0.3;
		Chassis.joint_right.Torque_behind = -0.3;

		I_left = -angle_pid_left.out + turn_pid_left.out;
		I_right = angle_pid_right.out + turn_pid_right.out;

		init_time++;
	}

	if (chassis_state == 3 || chassis_state > 4)
	{
		LegPID_Calc();
		LQR_Calc();
		if (offtimecount[0] > 2 && chassis_state < 4)
			I_left = 0;
		else
			I_left = -(T_left /*+dx_left/TORQUE_W*/) * TORQUE_K + turn_pid_left.out;
		if (offtimecount[1] > 2 && chassis_state < 4)
			I_right = 0;
		else
			I_right = (T_right /*+dx_right/TORQUE_W*/) * TORQUE_K + turn_pid_right.out;
	}
	if (chassis_state == 4)
	{
		Chassis.joint_left.Torque_forward = -0.3;
		Chassis.joint_left.Torque_behind = 0.3;
		Chassis.joint_right.Torque_forward = 0.3;
		Chassis.joint_right.Torque_behind = -0.3;

		I_left = -stop_pid.out + turn_pid_left.out;
		I_right = stop_pid.out + turn_pid_right.out;
	}

	if (I_right > M3508_MAX_OUT)
		I_right = M3508_MAX_OUT;
	if (I_right < -M3508_MAX_OUT)
		I_right = -M3508_MAX_OUT;
	if (I_left > M3508_MAX_OUT)
		I_left = M3508_MAX_OUT;
	if (I_left < -M3508_MAX_OUT)
		I_left = -M3508_MAX_OUT;
	FEET_CONTROL((int)(I_left), (int)(I_right));
}

void ForwardPID_Calc(void)
{

	//		if(offtimecount[0]<2&&Chassis.Mode!=NOFORCE)
	//				PID_init(&angle_pid_left,PID_POSITION,Angle_PID,M3508_MAX_OUT,M3508_MAX_IOUT);
	//		else
	//				PID_init(&angle_pid_left,PID_POSITION,Noforce_PID,M3508_MAX_OUT,M3508_MAX_IOUT);
	//		if(offtimecount[1]<2&&Chassis.Mode!=NOFORCE)
	//				PID_init(&angle_pid_right,PID_POSITION,Angle_PID,M3508_MAX_OUT,M3508_MAX_IOUT);
	//		else
	//				PID_init(&angle_pid_right,PID_POSITION,Noforce_PID,M3508_MAX_OUT,M3508_MAX_IOUT);

	if (chassis_state == 1)
	{
		PID_calc(&stop_pid, Chassis.LeftFootMotorMeasure.speed, 0);
		PID_calc(&stop_pid, Chassis.RightFootMotorMeasure.speed, 0);
	}
	if (chassis_state == 2)
	{
		float x = target_x_filter.out, dx = Chassis.Speed_KF.FilteredValue[1];
		float tht = -PID_calc(&speed_pid, dx + Chassis.vx, 0);
		PID_Calc(&angle_pid_left, Chassis.IMU.Angle[1], Chassis.IMU.Gyro[1], tht);
		PID_Calc(&angle_pid_right, Chassis.IMU.Angle[1], Chassis.IMU.Gyro[1], tht);
	}

	if (chassis_state == 4)
	{
		PID_calc(&stop_pid, Chassis.LeftFootMotorMeasure.speed, 0);
		PID_calc(&stop_pid, Chassis.RightFootMotorMeasure.speed, 0);
	}
}

void TurnPID_Calc(void)
{
	if (offtimecount[0] < 2 && offtimecount[0] < 2 && Chassis.Mode != NOFORCE)
		PID_init(&follow_pid, PID_POSITION, Follow_PID, FOLLOW_MAX_OUT, FOLLOW_MAX_IOUT);
	else
		PID_init(&follow_pid, PID_POSITION, Noforce_PID, FOLLOW_MAX_OUT, FOLLOW_MAX_IOUT);
	if (offtimecount[0] < 2 && Chassis.Mode != NOFORCE)
		PID_init(&turn_pid_left, PID_POSITION, Turn_left_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	else
		PID_init(&turn_pid_left, PID_POSITION, Noforce_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	if (offtimecount[1] < 2 && Chassis.Mode != NOFORCE)
		PID_init(&turn_pid_right, PID_POSITION, Turn_right_PID, M3508_MAX_OUT, M3508_MAX_IOUT);
	else
		PID_init(&turn_pid_right, PID_POSITION, Noforce_PID, M3508_MAX_OUT, M3508_MAX_IOUT);

	if (Chassis.Mode == NOFORCE)
	{
		turn_pid_left.out = 0;
		turn_pid_right.out = 0;
		return;
	}
	if (Chassis.Mode == ROTING)
	{
		if (fabs(Chassis.wz) < fabs(ROTING_SPEED))
			Chassis.wz += ROTING_SPEED * 0.0025f;
	}
	if (Chassis.Mode == FOLLOW || Chassis.Mode == JUMP || Chassis.Mode == DASH || Chassis.Mode == HIGHSPEED)
	{
		fp32 follow_angle = loop_fp32_constrain(FOLLOW_ANGLE, Chassis.YawMotorMeasure.angle - 180.0f, Chassis.YawMotorMeasure.angle + 180.0f);
		Chassis.wz = PID_calc(&follow_pid, Chassis.YawMotorMeasure.angle, follow_angle) * 3.14159f / 180.0f;
	}
	if (Chassis.Mode == STOP)
	{
		fp32 follow_angle = loop_fp32_constrain(FOLLOW_ANGLE + 90.0f, Chassis.YawMotorMeasure.angle - 180.0f, Chassis.YawMotorMeasure.angle + 180.0f);
		Chassis.wz = PID_calc(&follow_pid, Chassis.YawMotorMeasure.angle, follow_angle) * 3.14159f / 180.0f;
	}
	PID_calc(&turn_pid_left, -Chassis.IMU.Gyro[0], -Chassis.wz);
	PID_calc(&turn_pid_right, -Chassis.IMU.Gyro[0], -Chassis.wz);
}

void LegPID_Calc(void)
{

	if (offtimecount[0] < 2 && offtimecount[0] < 2 && Chassis.Mode != NOFORCE)
		PID_init(&roll_pid, PID_POSITION, Roll_PID, 1, 1);
	else
		PID_init(&roll_pid, PID_POSITION, Noforce_PID, 1, 1);
	if (chassis_state == 3 || chassis_state > 4)
	{
		fp32 rollangle = Chassis.IMU.Angle[2];
		if (rollangle > 3.14159f / 2.0f)
			rollangle -= 3.14159f;
		if (rollangle < -3.14159f / 2.0f)
			rollangle += 3.14159f;
		fp32 len = PID_Calc(&roll_pid, rollangle, Chassis.IMU.Gyro[2], 0);

		if (fabs(len) > Chassis.target_length - 0.08f)
		{
			if (fabs(len) + 0.08f > 0.31f)
				len = 0.2f;
			Chassis.joint_left.target_length = fabs(len) + 0.08f + len;
			Chassis.joint_right.target_length = fabs(len) + 0.08f - len;
		}
		else
		{
			Chassis.joint_left.target_length = Chassis.target_length + len;
			Chassis.joint_right.target_length = Chassis.target_length - len;
		}
	}
	if (chassis_state == 6)
	{
		Chassis.joint_left.Force_command = 400 - 360.0f / 0.2f * (Chassis.joint_left.length_filter.out - 0.1f);
		Chassis.joint_right.Force_command = 400 - 360.0f / 0.2f * (Chassis.joint_right.length_filter.out - 0.1f);
	}
	else
	{
		Chassis.joint_left.Force_command = PID_Calc(&leg_pid_left, Chassis.joint_left.length_filter.out, Chassis.joint_left.dlength_filter.out, Chassis.joint_left.target_length);
		Chassis.joint_right.Force_command = PID_Calc(&leg_pid_right, Chassis.joint_right.length_filter.out, Chassis.joint_right.dlength_filter.out, Chassis.joint_right.target_length);
	}
}

void LQR_Calc(void)
{
	fp32 Tp_left, Tp_right;
	fp32 l0_left = Chassis.joint_left.length_filter.out;
	fp32 l0_right = Chassis.joint_right.length_filter.out;
	fp32 a_right = Chassis.joint_right.alpha_filter.out;
	fp32 da_right = Chassis.joint_right.dalpha_filter.out;
	fp32 a_left = Chassis.joint_left.alpha_filter.out;
	fp32 da_left = Chassis.joint_left.dalpha_filter.out;
	fp32 dx = Chassis.Speed_KF.FilteredValue[1];
	if (offtimecount[0] > 2 && chassis_state < 4)
	{
		T_right = 0;
		target_x_filter.out = 0;
		Tp_right = -(Calc_Poly(l0_right, 1, 0) * (a_right) + Calc_Poly(l0_right, 1, 1) * da_right + Calc_Poly(l0_right, 1, 4) * Chassis.IMU.Angle[1] + Calc_Poly(l0_right, 1, 5) * Chassis.IMU.Gyro[1]);
	}
	else
	{
		T_right = -(Calc_Poly(l0_right, 0, 0) * (a_right) + Calc_Poly(l0_right, 0, 1) * da_right + Calc_Poly(l0_right, 0, 2) * (target_x_filter.out) * (Chassis.Mode != ROTING) + Calc_Poly(l0_right, 0, 3) * (dx + Chassis.vx) + Calc_Poly(l0_right, 0, 4) * Chassis.IMU.Angle[1] + Calc_Poly(l0_right, 0, 5) * Chassis.IMU.Gyro[1]);
		Tp_right = -(Calc_Poly(l0_right, 1, 0) * (a_right) + Calc_Poly(l0_right, 1, 1) * da_right + Calc_Poly(l0_right, 1, 2) * (target_x_filter.out) * (Chassis.Mode != ROTING) + Calc_Poly(l0_right, 1, 3) * (dx + Chassis.vx) + Calc_Poly(l0_right, 1, 4) * Chassis.IMU.Angle[1] + Calc_Poly(l0_right, 1, 5) * Chassis.IMU.Gyro[1]) - (a_right - a_left) * BALANCE_KP + (da_right - da_left) * BALANCE_KD;
	}
	if (offtimecount[1] > 2 && chassis_state < 4)
	{
		T_left = 0;
		target_x_filter.out = 0;
		Tp_left = -(Calc_Poly(l0_left, 1, 0) * (a_left) + Calc_Poly(l0_left, 1, 1) * da_left + Calc_Poly(l0_left, 1, 4) * Chassis.IMU.Angle[1] + Calc_Poly(l0_left, 1, 5) * Chassis.IMU.Gyro[1]);
	}
	else
	{
		T_left = -(Calc_Poly(l0_left, 0, 0) * (a_left) + Calc_Poly(l0_left, 0, 1) * da_left + Calc_Poly(l0_left, 0, 2) * (target_x_filter.out) * (Chassis.Mode != ROTING) + Calc_Poly(l0_left, 0, 3) * (dx + Chassis.vx) + Calc_Poly(l0_left, 0, 4) * Chassis.IMU.Angle[1] + Calc_Poly(l0_left, 0, 5) * Chassis.IMU.Gyro[1]);
		Tp_left = -(Calc_Poly(l0_left, 1, 0) * (a_left) + Calc_Poly(l0_left, 1, 1) * da_left + Calc_Poly(l0_left, 1, 2) * (target_x_filter.out) * (Chassis.Mode != ROTING) + Calc_Poly(l0_left, 1, 3) * (dx + Chassis.vx) + Calc_Poly(l0_left, 1, 4) * Chassis.IMU.Angle[1] + Calc_Poly(l0_left, 1, 5) * Chassis.IMU.Gyro[1]) + (a_right - a_left) * BALANCE_KP - (da_right - da_left) * BALANCE_KD;
	}
	CommandCalc(&(Chassis.joint_left), Chassis.joint_left.Force_command, -Tp_left);
	CommandCalc(&(Chassis.joint_right), Chassis.joint_right.Force_command, -Tp_right);
}