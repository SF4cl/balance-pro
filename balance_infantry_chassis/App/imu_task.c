#include "imu_task.h"



		float Gyro[3] = {0.0f};
		float Acc[3] = {0.0f};		





void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */

void ImuTask_Entry(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
	
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		

    while(BMI088_init())
    {
        osDelay(100);
    }
		Chassis.IMU.Quat[0]=1.0f;Chassis.IMU.Quat[1]=0;Chassis.IMU.Quat[2]=0;Chassis.IMU.Quat[3]=0;
    IMU_QuaternionEKF_Init(10, 0.01,10000000, 0.9996, 0);
		osDelay(100);
    /* Infinite loop */
    while(1)
    {

        BMI088_read(Gyro, Acc, &(Chassis.IMU.temp));
				
				Chassis.IMU.Baccel[0]=-Acc[1];
				Chassis.IMU.Baccel[1]=Acc[0]+Gyro[2]*Gyro[2]*0.152475f;
				Chassis.IMU.Baccel[2]=-Acc[2];
				Chassis.IMU.Gyro[0]=Gyro[1];
				Chassis.IMU.Gyro[1]=-Gyro[0];
				Chassis.IMU.Gyro[2]=Gyro[2];
				//MahonyAHRSupdateIMU(Chassis.IMU.Quat,Chassis.IMU.Gyro[0],Chassis.IMU.Gyro[1],Chassis.IMU.Gyro[2],Chassis.IMU.Baccel[0],Chassis.IMU.Baccel[1],Chassis.IMU.Baccel[2]);

			  IMU_QuaternionEKF_Update(Chassis.IMU.Gyro[0],Chassis.IMU.Gyro[1],Chassis.IMU.Gyro[2],Chassis.IMU.Baccel[0],Chassis.IMU.Baccel[1],Chassis.IMU.Baccel[2],0.001f);			
				memcpy(Chassis.IMU.Quat,QEKF_INS.q,sizeof(Chassis.IMU.Quat));
				GetAngle(Chassis.IMU.Quat,Chassis.IMU.Angle+INS_YAW_ADDRESS_OFFSET,Chassis.IMU.Angle+INS_PITCH_ADDRESS_OFFSET,Chassis.IMU.Angle+INS_ROLL_ADDRESS_OFFSET);			
			
				Chassis.IMU.Gyro[0]=-Gyro[2];
				Chassis.IMU.Gyro[1]=Gyro[0];
				Chassis.IMU.Gyro[2]=-Gyro[1]; 			


        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}



