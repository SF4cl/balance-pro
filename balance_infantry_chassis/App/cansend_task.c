/*
* ���������������̨������Ϣ
* ��������������ȼ�����ʵʱ�Ե�
* ���⣺Ŀǰ������Ϣ�����ж���ᵼ��FIFO���������Ӷ�����ʧ��
* ���˼·����������FIFO���������ֱ��Ӧ�ߵ����ȼ��������ȼ����������������ȼ�FIFO��
*			ÿ�����ȷ���ֱ�������ȼ���FIFO��ֱ���䷢����ϲŷ��͵����ȼ�FIFO
* ʵ�ֲ��֣�������FIFO���ִ���ʱ��ʵ�����ƣ����ǲ��ܼ���Ƿ�ɹ�����
* �޸ģ�1.��������ͬ������Ҫע����ǣ�һ��������FIFO���������Լ�һ������ϵͳ���ݻ�������
*			Ŀǰ˼·Ϊ����FIFO��һֱ�ڽ����ģ���������������ʱ������FIFO��գ�ͬʱ�����ݴ��뷢��FIFO��
*			�Լ�����ϵͳ���ݻ���������ʱ����������𣬷����������С�����FIFO���ʱ������������𣬽����������С�
*			�м����FIFOʼ�������У��ɶ�����������á�
*/
#include "cmsis_os.h"
#include "cansend_task.h"
#include "tim.h"
#include "fdcan.h"
#include "RefereeBehaviour.h"


uint8_t DataBuffer[11];
extern osThreadId CanSendHandle;
extern osThreadId RefereeHandle;

static FDCAN_TxHeaderTypeDef can_tx_message;

static void Can_Send(FDCAN_HandleTypeDef *hcan,uint32_t id,uint8_t lenth,uint8_t *buffer);
void CanSendThread(void *argument)
{
	uint8_t SendLenth;
	uint32_t SendId;
	while(1)
	{
		if(Pop(SendBuffer,(SendBuffer+1),DataBuffer) == 1)
		{
			SendId = (uint32_t)(DataBuffer[0]<<8|DataBuffer[1]);

			SendLenth = DataBuffer[2];

			Can_Send(&hfdcan2,SendId,SendLenth,&DataBuffer[3]);
		}

		osDelay(1);
	}
}

static void Can_Send(FDCAN_HandleTypeDef *hcan,uint32_t id,uint8_t lenth,uint8_t *buffer)
{

    can_tx_message.Identifier=id;                           //32λID
    can_tx_message.IdType=FDCAN_STANDARD_ID;                  //��׼ID
    can_tx_message.TxFrameType=FDCAN_DATA_FRAME;              //����֡
    can_tx_message.DataLength=FDCAN_DLC_BYTES_8;              //���ݳ���
    can_tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    can_tx_message.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
    can_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    can_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    can_tx_message.MessageMarker=0;      

    
    /* ������ݵ�TX FIFO */
    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &can_tx_message, buffer);
}

