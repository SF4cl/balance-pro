#include "feet_motor.h"

FDCAN_TxHeaderTypeDef  can_tx_message;
uint8_t              MotorSendBuffer[8];




void FEET_CONTROL(int16_t FEET_MOTOR_LEFT, int16_t FEET_MOTOR_RIGHT)
{
		
    MotorSendBuffer[2 * (FEET_MOTOR1_RECEIVE_ID - 0x201)] = FEET_MOTOR_LEFT >> 8;
    MotorSendBuffer[2 * (FEET_MOTOR1_RECEIVE_ID - 0x201) + 1] = FEET_MOTOR_LEFT;
    MotorSendBuffer[2 * (FEET_MOTOR2_RECEIVE_ID - 0x201)] = FEET_MOTOR_RIGHT >> 8;
    MotorSendBuffer[2 * (FEET_MOTOR2_RECEIVE_ID - 0x201) + 1] = FEET_MOTOR_RIGHT; 
  

    can_tx_message.Identifier=FEET_MOTOR_TRANSMIT_ID;                           //32位ID
    can_tx_message.IdType=FDCAN_STANDARD_ID;                  //标准ID
    can_tx_message.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    can_tx_message.DataLength=FDCAN_DLC_BYTES_8;              //数据长度
    can_tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    can_tx_message.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    can_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    can_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    can_tx_message.MessageMarker=0;      

    
    /* 添加数据到TX FIFO */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_message, MotorSendBuffer);
		
		
		
} 

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
			{
					Error_Handler();
			}
			if(hfdcan==&hfdcan1)
			{
					switch(RxHeader.Identifier)
					{
							case FEET_MOTOR1_RECEIVE_ID:
							{
									get_feet_motor_measure(&(Chassis.LeftFootMotorMeasure),RxData);
									Chassis.LeftFootMotorMeasure.speed=-Chassis.LeftFootMotorMeasure.speed_rpm / Reduction_ratio * Wheel_diameter / 60.0f * 3.14159f * 2.0f;
																		
									break;
							}
							case FEET_MOTOR2_RECEIVE_ID:
							{		
									get_feet_motor_measure(&(Chassis.RightFootMotorMeasure),RxData);		
									Chassis.RightFootMotorMeasure.speed=Chassis.RightFootMotorMeasure.speed_rpm / Reduction_ratio * Wheel_diameter / 60.0f * 3.14159f * 2.0f;
									break;
							}							
							case CMSRecceiveID:
							{
								int16_t i = ((uint16_t)RxData[2] << 8| RxData[3]);
								int16_t v = ((uint16_t)RxData[0] << 8| RxData[1]);
								CMS_Data.cms_cap_i = int16_to_float(i,32000, -32000,20, -20);
								CMS_Data.cms_cap_v = int16_to_float(v,32000, -32000,30, 0);
								CMS_Data.cms_status = ((uint16_t)RxData[4] << 8| RxData[5]);
								break;
							}						
					}
				}
			
				
							
			if(hfdcan==&hfdcan2)
			{
					switch(RxHeader.Identifier)
					{							
						
							case YawMotorId:
							{
									get_yaw_motor_measure(&(Chassis.YawMotorMeasure),RxData);
									break;
							}
							case DefaulPTZRequestAndStatusId:
							{
									memcpy(&PTZ,RxData,sizeof(PTZ_t));
									break;
							}
							case 0x14f:
							{
									memcpy(&UImessage,RxData,sizeof(UImessage));
							}
							case 0x106:
							{
									memcpy(&Aim,RxData,sizeof(Aim));
							}
							default:
							{
									break;
							}
					}
			}			
  }
}

