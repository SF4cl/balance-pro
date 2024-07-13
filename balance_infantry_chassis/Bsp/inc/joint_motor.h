#ifndef JOINT_MOTOR_H
#define JOINT_MOTOR_H

#include "usart.h"
#include "Setting.h"
#include "main.h"
#include "string.h"
#include "Struct_typedef.h"
#include "MotionAnalysis.h"
#include "crc32.h"



extern USART_SENDRECEIVETYPE Usart2Type;
extern USART_SENDRECEIVETYPE Usart3Type;

extern void Stand_left1_t(double T);
extern void Stand_right1_t(double T);
extern void Stand_left2_t(double T);
extern void Stand_right2_t(double T);
extern void Joint_motor_init(void);
extern void Joint_motor_set_mode(int MODE);
extern void modify_data(MOTOR_send *motor_s);
extern bool extract_data(uint8_t id,MOTOR_recv* motor_r);
extern void Change(float angle,JOINT_ID id);


#endif

