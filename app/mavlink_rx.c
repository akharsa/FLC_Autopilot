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
mavlink_message_t msg;
mavlink_status_t status;

xSemaphoreHandle DataSmphr;

uint8_t rx_led = 0;

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

			switch(msg.msgid){

				case MAVLINK_MSG_ID_HEARTBEAT:
					if (rx_led==0){
						qLed_TurnOn(STATUS_LED);
						rx_led = 1;
					}else{
						qLed_TurnOff(STATUS_LED);
						rx_led = 0;
					}
					break;

				case MAVLINK_MSG_ID_COMMAND_LONG:

					switch (mavlink_msg_command_long_get_command(&msg)){
						case MAV_CMD_NAV_TAKEOFF:
							quadrotor.mavlink_system.state = MAV_STATE_ACTIVE;
							quadrotor.mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
							break;
						case MAV_CMD_NAV_LAND:
							quadrotor.mavlink_system.state = MAV_STATE_STANDBY;
							quadrotor.mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
							break;
						case MAV_CMD_COMPONENT_ARM_DISARM:
							if (mavlink_msg_command_long_get_param1(&msg)==1){
							}else{
							}
							break;
						default:
							break;
					}

					// EXECUTE ACTION
					break;
				case MAVLINK_MSG_ID_MANUAL_CONTROL:
					mavlink_msg_manual_control_decode(&msg,&quadrotor.mavlink_control);
					if ((quadrotor.mavlink_control.buttons & 256)==0){
						quadrotor.mavlink_system.state = MAV_STATE_STANDBY;
						quadrotor.mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
					}
					break;

				default:
					//Do nothing
					break;
				}
		}
	}

	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}


