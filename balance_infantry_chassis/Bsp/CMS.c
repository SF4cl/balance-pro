#include "CMS.h"
#include "fdcan.h"
#include "CanPacket.h"
#include "Setting.h"


//CMS_t CMS;
CMS_Data_t CMS_Data;

static FDCAN_TxHeaderTypeDef  cms_buffer_tx_message;		
static uint8_t              cms_buffer_can_send_data[2];
static FDCAN_TxHeaderTypeDef  cms_power_tx_message;
static uint8_t              cms_power_can_send_data[8];
 //指令发送


//void CMS_Referee_Send(uint16_t charge_limit, uint8_t enable)
//{
//        uint32_t send_mail_box;
//    can_tx_message.StdId = CMSCurrentSendID;
//    can_tx_message.IDE = CAN_ID_STD;
//    can_tx_message.RTR = CAN_RTR_DATA;
//    can_tx_message.DLC = 0x03;
//    can_send_data[0] = charge_limit >> 8;
//    can_send_data[1] = charge_limit;
//    can_send_data[2] = enable;
//    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
//}


//send buffer power
void CMS_BUFFER_SEND(int16_t buffer)
{
    uint32_t send_mail_box;
    cms_buffer_can_send_data[0] = (buffer >> 8);
    cms_buffer_can_send_data[1] = buffer;
    cms_buffer_tx_message.Identifier=CMSBufferPowerSendID;                           //32位ID
    cms_buffer_tx_message.IdType=FDCAN_STANDARD_ID;                  //标准ID
    cms_buffer_tx_message.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    cms_buffer_tx_message.DataLength=FDCAN_DLC_BYTES_2;              //数据长度
    cms_buffer_tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    cms_buffer_tx_message.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    cms_buffer_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    cms_buffer_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    cms_buffer_tx_message.MessageMarker=0;      

    
    /* 添加数据到TX FIFO */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &cms_buffer_tx_message, cms_buffer_can_send_data);
		
}

//send data
void CMS_POWER_SEND(int16_t input_power_limit,int16_t output_powe_limit,int16_t cap_power_limit,int16_t cap_control)
{
    uint32_t send_mail_box;
    cms_power_can_send_data[0] = (input_power_limit >> 8);
    cms_power_can_send_data[1] = input_power_limit;
	cms_power_can_send_data[2] = (output_powe_limit >> 8);
    cms_power_can_send_data[3] = output_powe_limit;
	cms_power_can_send_data[4] = (cap_power_limit >> 8);
    cms_power_can_send_data[5] = cap_power_limit;
	cms_power_can_send_data[6] = (cap_control >> 8);
    cms_power_can_send_data[7] = cap_control;
    cms_power_tx_message.Identifier=CMSDateSendID;                           //32位ID
    cms_power_tx_message.IdType=FDCAN_STANDARD_ID;                  //标准ID
    cms_power_tx_message.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    cms_power_tx_message.DataLength=FDCAN_DLC_BYTES_8;              //数据长度
    cms_power_tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    cms_power_tx_message.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    cms_power_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    cms_power_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    cms_power_tx_message.MessageMarker=0;      

    
    /* 添加数据到TX FIFO */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &cms_power_tx_message, cms_power_can_send_data);
		
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
	int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
	//加0.5使向下取整变成四舍五入
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}