#include "MotionAnalysis.h"



float Sqrt(float x)
{
		float x_in,y_out;
		x_in=x;
		arm_sqrt_f32(x_in,&y_out);
		return y_out;
}
float Transform(float x)
{
		if(x>0)	return Sqrt(0.5f*x);
		return Sqrt(0.5f*-x);
}

void Joint_init(JOINT_t *joint)
{
		first_order_filter_init(&joint->alpha_filter,1.0f/IMUTask_frequency,FILTER_PARAMETER_JOINT_ALPHA);	
		first_order_filter_init(&joint->dalpha_filter,1.0f/IMUTask_frequency,FILTER_PARAMETER_JOINT_DALPHA);	
		first_order_filter_init(&joint->length_filter,1.0f/IMUTask_frequency,FILTER_PARAMETER_JOINT_LENGTH);	
		first_order_filter_init(&joint->dlength_filter,1.0f/IMUTask_frequency,FILTER_PARAMETER_JOINT_DLENGTH);		
		first_order_filter_init(&joint->force_feedback_filter,1.0f/IMUTask_frequency,FILTER_PARAMETER_JOINT_FORCE);		
		Joint_motor_init();
}
fp32 k,th_sin,th_cos,F_r,Tp_r;
void AttitudeCalc(JOINT_t *joint,IMUData_t *imu)
{
		fp32 u0,u2,u3;
		fp32 xd,yd,xb,yb,lbd,A10,B10,C10,xc,yc,l0;
		fp32 dxc,dyc;		
		
		fp32 cosu1,cosu4,sinu1,sinu4,sinu2u3;
	
		fp32 l1=Thigh,l2=Calf,W=Innominate,mw=0;
		fp32 u1=joint->joint_behind.motor_r.angle_filter.out;
		fp32 du1=joint->joint_behind.motor_r.speed_filter.out;
		fp32 u4=joint->joint_forward.motor_r.angle_filter.out;
		fp32 du4=joint->joint_forward.motor_r.speed_filter.out;
		cosu1=arm_cos_f32(u1),cosu4=arm_cos_f32(u4),sinu1=arm_sin_f32(u1),sinu4=arm_sin_f32(u4);

		xb=l1*cosu1-W/2.0f;
		yb=l1*sinu1;
		xd=W/2.0f+l1*cosu4;
		yd=l1*sinu4;
		lbd=Sqrt((xd-xb)*(xd-xb)+(yd-yb)*(yd-yb));
		A10=2*l2*(xd-xb);
		B10=2*l2*(yd-yb);
		C10=lbd*lbd;
		u2=2*atan2((B10+Sqrt(A10*A10+B10*B10-C10*C10)),(A10+C10));
		u3=atan2(yb-yd+l2*arm_sin_f32(u2),xb-xd+l2*arm_cos_f32(u2));
		sinu2u3=arm_sin_f32(u2-u3);
		xc=xb+l2*arm_cos_f32(u2);
		yc=yb+l2*arm_sin_f32(u2);
		u0=atan2(yc,xc);
		l0=Sqrt(xc*xc+yc*yc);
		dxc=l1*arm_sin_f32(u1-u2)*arm_sin_f32(u3)/sinu2u3*du1+l1*arm_sin_f32(u3-u4)*arm_sin_f32(u2)/sinu2u3*du4;
		dyc=l1*arm_sin_f32(u1-u2)*arm_cos_f32(u3)/sinu2u3*du1-l1*arm_sin_f32(u3-u4)*arm_cos_f32(u2)/sinu2u3*du4;

		fp32 a=-1.5708f-(imu->Angle[1]-u0);
		fp32 da=-(dxc*yc-xc*dyc)/(xc*xc+yc*yc)-imu->Gyro[1];
		fp32 dl0=(xc*dxc+yc*dyc)/l0;	
		fp32 l_da=joint->dalpha_filter.out;
		fp32 l_dl0=joint->dlength_filter.out;
		
		first_order_filter_cali(&(joint->length_filter),l0);
		first_order_filter_cali(&(joint->dlength_filter),dl0);
		first_order_filter_cali(&(joint->alpha_filter),a);
		first_order_filter_cali(&(joint->dalpha_filter),da);		

		joint->alpha=a;
		joint->dalpha=da;
		joint->length=l0;
		joint->dlength=dl0;
		

		joint->ja=l1*arm_sin_f32(u0-u3)*arm_sin_f32(u1-u2)/-sinu2u3;
		joint->jb=l1*arm_cos_f32(u0-u3)*arm_sin_f32(u1-u2)/(l0*-sinu2u3);
		joint->jc=l1*arm_sin_f32(u0-u2)*arm_sin_f32(u3-u4)/-sinu2u3;
		joint->jd=l1*arm_cos_f32(u0-u2)*arm_sin_f32(u3-u4)/(l0*-sinu2u3);
		
		fp32 T1_r=joint->joint_forward.motor_r.T,T2_r=joint->joint_behind.motor_r.T;
		
		th_cos=arm_cos_f32(a);
		th_sin=arm_sin_f32(a);
		k=joint->ja*joint->jd-joint->jb*joint->jc;
		F_r=(joint->jd*T1_r-joint->jb*T2_r)*9.1f/k;
		Tp_r=(-joint->jc*T1_r+joint->ja*T2_r)*9.1f/k;
		fp32 P=F_r*th_cos+Tp_r*th_sin/l0;
		
		fp32 Fn=P+mw*(imu->Baccel[2]+th_cos*da*da*l0+l0*th_sin*(da-l_da)*1000.0f+2*dl0*da*th_sin-th_cos*(dl0-l_dl0)*1000.0f);
		first_order_filter_cali(&joint->force_feedback_filter,Fn);	
		
}

uint8_t CommandCalc(JOINT_t *joint,fp32 Force,fp32 Torque)
{
		if(joint->joint_behind.motor_r.control_id==LEFT2&&joint->joint_forward.motor_r.control_id==LEFT1)
		{
				joint->Torque_forward=-(joint->ja*Force+joint->jb*Torque)/9.1f;
				joint->Torque_behind=-(joint->jc*Force+joint->jd*Torque)/9.1f;				
				return 1;
		}
		if(joint->joint_behind.motor_r.control_id==RIGHT2&&joint->joint_forward.motor_r.control_id==RIGHT1)
		{
				joint->Torque_forward=(joint->ja*Force+joint->jb*Torque)/9.1f;
				joint->Torque_behind=(joint->jc*Force+joint->jd*Torque)/9.1f;				
				return 2;
		}		
		return 0;
}
