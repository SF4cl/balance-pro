#include "joint_motor.h"

USART_SENDRECEIVETYPE Usart2Type ;//__attribute__((section(".ARM.__at_0x24000000")));
USART_SENDRECEIVETYPE Usart3Type ;//__attribute__((section(".ARM.__at_0x24000000")));





float KLEFT1=-4.1280303f,KLEFT2=3.13318396f,KRIGHT1=-3.4623909f,KRIGHT2=4.16107988f;

float Calc_pos(float pos,JOINT_ID id)
{
	if(id==LEFT1)
		return 3.14159f+pos/9.1f+KLEFT1;
	if(id==LEFT2)
		return pos/9.1f+KLEFT2;
	if(id==RIGHT1)
		return 3.14159f-pos/9.1f+KRIGHT1;
	if(id==RIGHT2)
		return -pos/9.1f+KRIGHT2;
	return 0;
}

void Change(float angle,JOINT_ID id)
{
		if(id==LEFT1)
				KLEFT1+=-0.52359f-angle;
		if(id==RIGHT1)
				KRIGHT1+=-0.52359f-angle;
		if(id==LEFT2)
				KLEFT2+=3.66519f-angle;
		if(id==RIGHT2)
				KRIGHT2+=3.66519f-angle;
}


void Joint_motor_set_mode(int MODE)
{		
    Chassis.joint_left.joint_behind.motor_s.mode = MODE;
    Chassis.joint_left.joint_forward.motor_s.mode = MODE;
    Chassis.joint_right.joint_behind.motor_s.mode = MODE;
    Chassis.joint_right.joint_forward.motor_s.mode = MODE;

}
void Joint_motor_init(void)
{
    Chassis.joint_left.joint_forward.motor_s.mode = 0;
		Chassis.joint_left.joint_forward.motor_s.id=JOINT_LEFT1_ID;
		Chassis.joint_left.joint_forward.motor_r.motor_id=JOINT_LEFT1_ID;
		Chassis.joint_left.joint_forward.motor_r.control_id=LEFT1;
	
    Chassis.joint_left.joint_behind.motor_s.mode = 0;
		Chassis.joint_left.joint_behind.motor_s.id=JOINT_LEFT2_ID;
		Chassis.joint_left.joint_behind.motor_r.motor_id=JOINT_LEFT2_ID;
		Chassis.joint_left.joint_behind.motor_r.control_id=LEFT2;
	
    Chassis.joint_right.joint_forward.motor_s.mode = 0;
		Chassis.joint_right.joint_forward.motor_s.id=JOINT_RIGHT1_ID;
		Chassis.joint_right.joint_forward.motor_r.motor_id=JOINT_RIGHT1_ID;
		Chassis.joint_right.joint_forward.motor_r.control_id=RIGHT1;
	
    Chassis.joint_right.joint_behind.motor_s.mode = 0;
		Chassis.joint_right.joint_behind.motor_s.id=JOINT_RIGHT2_ID;
		Chassis.joint_right.joint_behind.motor_r.motor_id=JOINT_RIGHT2_ID;
		Chassis.joint_right.joint_behind.motor_r.control_id=RIGHT2;		
		first_order_filter_init(&(Chassis.joint_left.joint_forward.motor_r.angle_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_ANGLE);	
		first_order_filter_init(&(Chassis.joint_left.joint_behind.motor_r.angle_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_ANGLE);	
		first_order_filter_init(&(Chassis.joint_right.joint_forward.motor_r.angle_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_ANGLE);	
		first_order_filter_init(&(Chassis.joint_right.joint_behind.motor_r.angle_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_ANGLE);	
		
		first_order_filter_init(&(Chassis.joint_left.joint_forward.motor_r.speed_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_SPEED);	
		first_order_filter_init(&(Chassis.joint_left.joint_behind.motor_r.speed_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_SPEED);	
		first_order_filter_init(&(Chassis.joint_right.joint_forward.motor_r.speed_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_SPEED);	
		first_order_filter_init(&(Chassis.joint_right.joint_behind.motor_r.speed_filter),1.0f/A1_Feedback_Frequency,FILTER_PARAMETER_JOINT_SPEED);			
}

void Stand_left1_t(double T)
{
	Chassis.joint_left.joint_forward.motor_s.K_P=0;
	Chassis.joint_left.joint_forward.motor_s.K_W=0;
	Chassis.joint_left.joint_forward.motor_s.Pos=0;
	Chassis.joint_left.joint_forward.motor_s.T=T;
	modify_data(&(Chassis.joint_left.joint_forward.motor_s));
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);
	memcpy(Usart2Type.TX_pData,(uint8_t*)&(Chassis.joint_left.joint_forward.motor_s.motor_send_data),A1_SEND_SIZE);
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);	
	HAL_UART_Transmit_DMA(&huart2,Usart2Type.TX_pData,A1_SEND_SIZE);    
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);

}

void Stand_left2_t(double T)
{
	Chassis.joint_left.joint_behind.motor_s.K_P=0;
	Chassis.joint_left.joint_behind.motor_s.K_W=0;
	Chassis.joint_left.joint_behind.motor_s.Pos=0;
	Chassis.joint_left.joint_behind.motor_s.T=T;
	modify_data(&(Chassis.joint_left.joint_behind.motor_s));
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);
	memcpy(Usart2Type.TX_pData,(uint8_t*)&(Chassis.joint_left.joint_behind.motor_s.motor_send_data),A1_SEND_SIZE);
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);
	HAL_UART_Transmit_DMA(&huart2,Usart2Type.TX_pData,A1_SEND_SIZE);    
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);	
}

void Stand_right1_t(double T)
{
	Chassis.joint_right.joint_forward.motor_s.K_P=0;
	Chassis.joint_right.joint_forward.motor_s.K_W=0;
	Chassis.joint_right.joint_forward.motor_s.Pos=0;
	Chassis.joint_right.joint_forward.motor_s.T=T;
	modify_data(&(Chassis.joint_right.joint_forward.motor_s));
  //SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	memcpy(Usart3Type.TX_pData,(uint8_t*)&(Chassis.joint_right.joint_forward.motor_s.motor_send_data),A1_SEND_SIZE);
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	HAL_UART_Transmit_DMA(&huart3,Usart3Type.TX_pData,A1_SEND_SIZE);  
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);
}

void Stand_right2_t(double T)
{
	Chassis.joint_right.joint_behind.motor_s.K_P=0;
	Chassis.joint_right.joint_behind.motor_s.K_W=0;
	Chassis.joint_right.joint_behind.motor_s.Pos=0;
	Chassis.joint_right.joint_behind.motor_s.T=T;
	modify_data(&(Chassis.joint_right.joint_behind.motor_s));
  //SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	memcpy(Usart3Type.TX_pData,(uint8_t*)&(Chassis.joint_right.joint_behind.motor_s.motor_send_data),A1_SEND_SIZE);
	//SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),34);
	HAL_UART_Transmit_DMA(&huart3,Usart3Type.TX_pData,A1_SEND_SIZE); 
  //SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),34);	
}

void modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T*256;
    motor_s->motor_send_data.Mdata.W = motor_s->W*128;
    motor_s->motor_send_data.Mdata.Pos = (float)((motor_s->Pos/6.2832)*16384.0);
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P*2048;
    motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*1024;
    
    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}
bool extract_data(uint8_t id,MOTOR_recv* motor_r)
{

		if(id==motor_r->motor_recv_data.head.motorID)
		{
			motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
			motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
			motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
			motor_r->T = (float)(((float)motor_r->motor_recv_data.Mdata.T) / 256.0f);
			motor_r->W = (float)(((float)motor_r->motor_recv_data.Mdata.W) / 128.0f);
			motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

			motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
			motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.Mdata.Pos) / 16384.0f;
			
			motor_r->gyro[0] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176f);
			motor_r->gyro[1] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176f);
			motor_r->gyro[2] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176f);
			
			motor_r->acc[0] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132f);
			motor_r->acc[1] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132f);
			motor_r->acc[2] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132f);		
			
			fp32 angle=Calc_pos(motor_r->Pos,motor_r->control_id);
			fp32 speed=motor_r->W / -9.1f;
			if(motor_r->control_id == LEFT1 || motor_r->control_id == LEFT2)
					speed *= -1 , motor_r->T *= -1;
			first_order_filter_cali(&(motor_r->angle_filter),angle);
			first_order_filter_cali(&(motor_r->speed_filter),speed);
			return true;
		}

		return false;

}
