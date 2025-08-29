/*
This main file has all the library functions and supporting enums and char arrays. 
The next step is replicating this functionality by including the external library files.
*/

#include "main.h"
#include <string.h>

typedef struct {
    uint32_t              TxMailbox;       /* The number of the mail box that transmitted the Tx message */
    CAN_TxHeaderTypeDef   TxHeader;        /* Header containing the information of the transmitted frame */
    uint8_t               TxData[8]; /* Buffer of the data to send */
    CAN_RxHeaderTypeDef   RxHeaderFIFO0;   /* Header containing the information of the received frame */
    uint8_t               RxDataFIFO0[8];  /* Buffer of the received data */

    char hardware[20]; //hardware type from the incoming message
    char dataType[20]; //datatype from the incoming
    int device_id; //id of the device this is operating on

    double data; //data from incoming message
    uint8_t filter_bank; //keep track of which filter bank to fill next; we want to keep it between 0 and 14

    CAN_HandleTypeDef *hcan;

}CANBUS;

typedef union{
	double num;
	uint8_t arr[8];
}DoubleCaster;

enum devices{
	HS1,
	HS2,
	HS3
};

enum types{
	PRESSURE,
	RPM
};

const char *devices []= {
    "HS1",
    "HS2",
    "HS3"
};

const char *types [] = {
	"Pressure",
	"RPM"
};


void CAN_QuickSetup(CANBUS *can, int hardware, CAN_HandleTypeDef *can_obj); //implemented

void CAN_Run(CANBUS *can); //implemented

double CAN_GetData(CANBUS *can); //implemented

char* CAN_GetDataType(CANBUS *can); //implemented

char* CAN_GetHardware(CANBUS *can); //implemented

void CAN_AddFilterDeviceData(CANBUS *can, int device_id, int data_type); //TODO

void CAN_AddFilterDevice(CANBUS *can, int device_id); //TODO

void CAN_Send(CANBUS *can, double message, uint8_t data_type); //implemented

void CAN_Interrupt_Helper(CANBUS *can); //implemented


void SystemClock_Config(void);
static void MX_GPIO_Init(void);

CAN_HandleTypeDef hcan1;
CANBUS can1;

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();


  CAN_QuickSetup(&can1, HS3, &hcan1);
  CAN_Run(&can1);

  while (1)
  {

	  CAN_Send(&can1, 54.4, PRESSURE);
	  HAL_Delay(100);

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)

{
    /* Get RX message from FIFO0 and fill the data on the related FIFO0 user declared header
       (RxHeaderFIFO0) and table (RxDataFIFO0) */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &(can1.RxHeaderFIFO0), can1.RxDataFIFO0) != HAL_OK)
    {
        /* Reception Error */
       Error_Handler();
    }else{
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    	CAN_Interrupt_Helper(&can1);
    }

}

void Error_Handler(void)
{
  __disable_irq();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  while(1);
}

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

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
