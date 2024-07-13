#ifndef _STRUCT_TYPEDEF_H
#define _STRUCT_TYPEDEF_H

#include "Config.h"
#include "unitreeMotor.h"
#include "user_lib.h"
#include "kalman filter.h"

//#include "main.h"
#define A1_RECEIVE_SIZE						78
#define A1_SEND_SIZE							34




typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 min_out;  //最小输出 
    fp32 max_iout; //最大积分输出
    fp32 min_iout; //最小积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

typedef struct
{
	MOTOR_send motor_s;
	MOTOR_recv motor_r;
} joint_motor;

typedef struct{
    uint16_t    ecd;
    int16_t     speed_rpm;
    int16_t     given_current;
    uint8_t     temperature;
    int16_t     last_ecd;
		int16_t			last_speed_rpm;
		fp32 mileage; 
	  fp32 angle;
		fp32 speed;
}feet_motor_measure_t;


typedef struct {

		joint_motor joint_forward,joint_behind;
		
		first_order_filter_type_t force_feedback_filter;
		first_order_filter_type_t alpha_filter,dalpha_filter;
		first_order_filter_type_t length_filter,dlength_filter;
		fp32 length,dlength,target_length,alpha,dalpha;
		fp32 Torque_forward,Torque_behind,Force_command;
	
		fp32 ja,jb,jc,jd;
	
		
}JOINT_t;


typedef struct
{
		fp32 Quat[4];
		fp32 Angle[3];
		fp32 Gyro[3];
		fp32 Baccel[3];
		fp32 temp;
}IMUData_t;

typedef enum
{
	NOFORCE,
	STOP,
	FOLLOW,
	ROTING,
	HIGHSPEED,
	JUMP,
	DASH
}ChassisMode_e;



typedef struct {
	ChassisMode_e Mode;
	uint8_t flag;
	fp32 vx,wz;
	fp32 target_length;
	
	JOINT_t joint_left,joint_right;
	feet_motor_measure_t LeftFootMotorMeasure,RightFootMotorMeasure;
	feet_motor_measure_t YawMotorMeasure;
	IMUData_t IMU;
	KalmanFilter_t Speed_KF;

}Chassis_t;

typedef struct
{
    pid_type_def    pid_inside;
    pid_type_def    pid_outside;
    
    fp32            s_set;
    fp32            s_fdb;
    fp32            v_set;
    fp32            v_fdb;
    fp32            out;
} cascade_pid_t;


typedef struct
{
		uint8_t mode;
		uint8_t change_flag;
	
} Debug_flag_t;


typedef struct{
    fp32 *ptr;
    uint32_t offset;
    uint32_t size;
} LoopFifoFp32_t;


typedef struct
{
	uint8_t RX_flag:1;//IDLE receive flag

	uint16_t RX_Size;//receive length

	uint8_t RX_pData[A1_RECEIVE_SIZE+2];//DMA receive buffer

	uint8_t TX_pData[A1_SEND_SIZE];
}USART_SENDRECEIVETYPE;


extern Chassis_t Chassis;

#endif



