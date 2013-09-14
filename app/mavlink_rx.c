/*
 * mavlink_rx.c
 *
 *  Created on: 14/09/2013
 *      Author: alan
 */


#include "quadrotor.h"
#include "DebugConsole.h"
#include "board.h"
#include "types.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mavlink.h"
#include "mavlink_bridge.h"

#include "qUART.h"

static int packet_drops = 0;
volatile mavlink_message_t msg;
volatile mavlink_status_t status;

xSemaphoreHandle DataSmphr;

void UART_Rx_Handler(uint8_t * buff, size_t sz);

void Communications(void * pvParameters){

	vSemaphoreCreateBinary(DataSmphr);

    if (DataSmphr==NULL){
    	halt();
    }

    while (qUARTStatus[UART_GROUNDCOMM]!=DEVICE_READY);

    qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);
    qUART_EnableRx(UART_GROUNDCOMM);

	for (;;){
		if (pdTRUE == xSemaphoreTake(DataSmphr,500/portTICK_RATE_MS)){
			// Tengo data nueva!
		}else{
			// Timeout to get a new joystick commands, values to 0
		}
	}
}


void UART_Rx_Handler(uint8_t * buff, size_t sz){
	uint32_t i;
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	for (i=0;i<sz;i++){
		if(mavlink_parse_char(MAVLINK_COMM_0, *(buff+i), &msg, &status)) {
			qUART_Send(UART_GROUNDCOMM,msg.payload64,msg.len);
			switch(msg.msgid){

				//case MAVLINK_MSG_ID_HEARTBEAT:
					// E.g. read GCS heartbeat and go into
					// comm lost mode if timer times out
				//	break;
				//case MAVLINK_MSG_ID_COMMAND_LONG:
					// EXECUTE ACTION
				//	break;
				default:
					//Do nothing
					break;
				}
		}
		//ret=qComms_ParseByte(&msg,*(buff+i));
	}

	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}


