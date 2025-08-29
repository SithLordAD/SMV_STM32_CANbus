#include "smv_canbus.h"
#include <string.h>
/*
IMPORTANT: 
This file is a just a copy and paste of the functions that were set up in the large main file; The library has not been tested in this form yet
The main file has enums and char arrays that also need to be included in their own file (we can reuse the files from the old library)
We need to confirm that we have all the includes we need
*/

void CAN_QuickSetup(CANBUS *can, int hardware, CAN_HandleTypeDef *can_obj){
	can->hcan = can_obj;


	can->device_id = hardware;
	can->filter_bank = 0;

	CAN_FilterTypeDef  sFilterConfig;

	can->hcan->Instance = CAN1;
	can->hcan->Init.Prescaler = 6;
	can->hcan->Init.Mode = CAN_MODE_NORMAL;
	can->hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
	can->hcan->Init.TimeSeg1 = CAN_BS1_9TQ;
	can->hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
	can->hcan->Init.TimeTriggeredMode = DISABLE;
	can->hcan->Init.AutoBusOff = ENABLE;
	can->hcan->Init.AutoWakeUp = DISABLE;
	can->hcan->Init.AutoRetransmission = ENABLE;
	can->hcan->Init.ReceiveFifoLocked = DISABLE;
	can->hcan->Init.TransmitFifoPriority = DISABLE;

	__HAL_RCC_CAN1_CLK_ENABLE();

	if (HAL_CAN_Init(can_obj) != HAL_OK)
	{
		Error_Handler();
	}

	sFilterConfig.SlaveStartFilterBank = 14;           /* Slave start bank Set only once. */

	sFilterConfig.FilterBank = 0;                      /* Select the filter number 0 */
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* Using ID mask mode .. */
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* .. in 32-bit scale */
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;                /* The filter is set to receive only the Standard ID frames */
	sFilterConfig.FilterMaskIdHigh = 0x0000;           /* Accept all the IDs .. except the Extended frames */
	sFilterConfig.FilterMaskIdLow = 0x0000;            /* The filter is set to check only on the ID format */
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; /* All the messages accepted by this filter will be received on FIFO1 */
	sFilterConfig.FilterActivation = ENABLE;           /* Enable the filter number 0 */


	if (HAL_CAN_ConfigFilter(can_obj, &sFilterConfig) != HAL_OK)
	{
	   /* Filter configuration Error */
	   Error_Handler();
	}
}

void CAN_Run(CANBUS *can){
	if (HAL_CAN_Start(can->hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(can->hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	{
		 /* Notification Error */
		 Error_Handler();
	}

    can->TxHeader.RTR = CAN_RTR_DATA;         /* The frames that will be sent are Data */
    can->TxHeader.DLC = 8;                    /* The frames will contain 8 data bytes */
    can->TxHeader.IDE = CAN_ID_STD;

    can->TxHeader.TransmitGlobalTime = DISABLE;

}

void CAN_Send(CANBUS *can, double message, uint8_t data_type){
	can->TxHeader.StdId = ((can->device_id & 0x0F) << 7) + data_type;

	DoubleCaster c;
	c.num = message;

	for (int j = 0; j < 8; j++){
		can->TxData[j] = c.arr[j];
	}

	/* It's mandatory to look for a free Tx mail box */
	while(HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) == 0) {} /* Wait till a Tx mailbox is free. Using while loop instead of HAL_Delay() */

	if (HAL_CAN_AddTxMessage(can->hcan, &(can->TxHeader), can->TxData, &(can->TxMailbox)) != HAL_OK) /* Send the CAN frame */
	{
		/* Transmission request Error */
		Error_Handler();
	}


}

double CAN_GetData(CANBUS *can){
	return can->data;
}

char* CAN_GetDataType(CANBUS *can){
	return can->dataType;
}

char* CAN_GetHardware(CANBUS *can){
	return can->hardware;
}

void CAN_Interrupt_Helper(CANBUS *can){
	DoubleCaster c;
	for (int i = 0; i<8; i++){
		c.arr[i] = can->RxDataFIFO0[i];
	}
	can->data = c.num;

	//assign hardware array
	uint8_t hardware_id = ((can->RxHeaderFIFO0.StdId)>>7)& 0x0F;
	memcpy(can->hardware, devices[hardware_id], strlen(devices[hardware_id])+1);


	//assign dataType array
	uint8_t dataType_id = (can->RxHeaderFIFO0.StdId)&0x0F;
	memcpy(can->dataType, types[dataType_id], strlen(types[dataType_id])+1);
}

void CAN_AddFilterDevice(CANBUS *can, int device_id){
    //TODO: Implement this function
}

void CAN_AddFilterDeviceData(CANBUS *can, int device_id, int data_type){
    //TODO: Implement this function
}
