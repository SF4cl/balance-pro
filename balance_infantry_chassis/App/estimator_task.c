#include "estimator_task.h"


void Speed_KF_Init(KalmanFilter_t *kf);




void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}



//fp32 x,dx;

void EstimatorTask_Entry(void const * argument)
{
		first_order_filter_type_t accel_filter;
		
		fp32 Eaccel[3];
		
		osDelay(100);  
		Speed_KF_Init(&(Chassis.Speed_KF));
		first_order_filter_init(&accel_filter,1.0f/IMUTask_frequency,FILTER_PARAMETER_ACCEL);	

		while(1)
		{
				float acc;
				BodyFrameToEarthFrame(Chassis.IMU.Baccel,Eaccel,Chassis.IMU.Quat);			
				acc=arm_cos_f32(Chassis.IMU.Angle[0])*-Eaccel[1]+arm_sin_f32(Chassis.IMU.Angle[0])*Eaccel[0];    
			
				Chassis.Speed_KF.MeasuredVector[0]=(Chassis.LeftFootMotorMeasure.speed+Chassis.RightFootMotorMeasure.speed)/2.0f;
				first_order_filter_cali(&accel_filter,acc);
				Chassis.Speed_KF.ControlVector[0]=accel_filter.out;			
				Kalman_Filter_Update(&(Chassis.Speed_KF));
//				x=Chassis.Speed_KF.FilteredValue[0];
//				dx=Chassis.Speed_KF.FilteredValue[1];
			
				AttitudeCalc(&(Chassis.joint_left),&(Chassis.IMU));
				AttitudeCalc(&(Chassis.joint_right),&(Chassis.IMU));
				
				osDelay(1); 
		}
		
	
}

void Speed_KF_Init(KalmanFilter_t *kf)
{
		float P_Init[4] =
						{
								1,		0,
								0,		1
						};
		float F_Init[4] =
						{
									1,		0.001f,
									0,				1
						}; 
		float Q_Init[4] =
						{
									0.5*0.001*0.001*0.001,			0.5*0.001*0.001,
									0.001*0.001,								0.001
						};
		float R_Init[1] =
						{
								 10
						}; 
		float H_Init[2] =
						{
								0,          1
						};    
		float B_Init[2] =
						{
								0.001*0.001*0.5,          0.001
						};   	
		float K_Init[2] =
						{
								 0,         0
						}; 						
			
    // 设置最小方差
    static float state_min_variance[2] = {0.1,10};
    
    // 开启自动调整
    kf->UseAutoAdjustment = 0;
    
    static uint8_t measurement_reference[1] = {1};
    
    static float measurement_degree[1] = {1};
    
    static float mat_R_diagonal_elements[1] = {1};
		
    // 设置矩阵值
    Kalman_Filter_Init(kf, 2, 1, 1,NULL,NULL,NULL,NULL);
    memcpy(kf->MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(kf->MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(kf->MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(kf->StateMinVariance, state_min_variance, sizeof(state_min_variance));       
    memcpy(kf->F_data, F_Init, sizeof(F_Init));
    memcpy(kf->Q_data, Q_Init, sizeof(Q_Init));
		memcpy(kf->R_data, R_Init, sizeof(R_Init));
		memcpy(kf->H_data, H_Init, sizeof(H_Init));
		memcpy(kf->B_data, B_Init, sizeof(B_Init));		
}