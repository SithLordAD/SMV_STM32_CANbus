#ifndef __SMVCANBUS_H
#define __SMVCANBUS_H

#include "stm32f4xx_hal.h"
#include "smv_board_enums.h"

typedef struct CANBUS CANBUS;
struct CANBUS {
    /* MEMBERS */
    /* DO NOT USE DIRECTLY UNLESS YOU KNOW WHAT YOU'RE DOING */
    // stm32 stuff.
    uint32_t              TxMailbox;       /* The number of the mail box that transmitted the Tx message */
    CAN_TxHeaderTypeDef   TxHeader;        /* Header containing the information of the transmitted frame */
    uint8_t               TxData[8]; /* Buffer of the data to send */
    CAN_RxHeaderTypeDef   RxHeaderFIFO0;   /* Header containing the information of the received frame */
    uint8_t               RxDataFIFO0[8];  /* Buffer of the received data */
    CAN_HandleTypeDef *hcan;
    CAN_FilterTypeDef  sFilterConfig;
    int device_id; // id of your board.

    // message data
    char hardware[20]; // hardware type from the incoming message
    char dataType[20]; // datatype from the incoming
    double data; // data from incoming message
    uint8_t filter_bank; //keep track of which filter bank to fill next; we want to keep it between 0 and 14

    /* METHODS */
    // use only these to interact with your CANBUS object.
    void (*init)(CANBUS*, int, CAN_HandleTypeDef*); /* initialize the CAN bus driver @param CANBUS* pointer to your CANBUS object @param int your board's ID. reference the enums. @param CAN_HandleTypeDef* pointer to your STM32 generated CAN handler */
    void (*begin)(CANBUS*); /* CAN bus begins running in normal mode @param CANBUS* pointer to your CANBUS object */
    double (*getData)(CANBUS*); /* get incoming message from CAN bus line @param CANBUS* pointer to your CANBUS object */
    char* (*getDataType)(CANBUS*); /* get data type of incoming message from CAN bus line @param CANBUS* pointer to your CANBUS object */
    char* (*getHardware)(CANBUS*); /* get data type of incoming message from CAN bus line @param CANBUS* pointer to your CANBUS object */
    void (*addFilterDevice)(CANBUS*, int); /* receive data only from the device specified, filters everything else @param CANBUS* pointer to your CANBUS object @param int the board ID to listen to. reference the enums. */
    void (*addFilterDeviceData)(CANBUS*, int, int); /* receive only the specified type data only from the device specified, filters everything else @param CANBUS* pointer to your CANBUS object @param int the board ID to listen to. reference the enums. @param int the datatype ID to listen to. reference the enums. */
    void (*send)(CANBUS*, double, uint8_t); /* send data to the CAN bus @param CANBUS* pointer to your CANBUS object @param double the data you want to send @param uint8_t the type of data you are sending. reference the enums. */

};

/*
Constructor workaround in C. Call this when creating your CAN object, and then proceed with the rest of the example.
*/
CANBUS CAN_new(void);

/*
- CubeMX defines a CAN interrupt handler when the programmer enables the interrupt in NVIC settings in ioc
- Any definition of the Fifo0PendingCallback function will override the default definition, if any
- The programmer will define the Fifo0PendingCallback function with GetRxMessage and then call this helper function inside

Purpose:
- Assign the new message to our RxData array
- Use DoubleCaster union to assign the byte array value to the data variable
- Assign the incoming header to our RxHeader object
- Extract the integer values of device_id and data_type encoded in the RxHeader.StdId
    - Look up which strings they are both associated with and assign the appropriate values to hardware[] and dataType[]

*/
void CAN_Interrupt_Helper(CANBUS *can);

#endif
