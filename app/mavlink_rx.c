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
#include "parameters.h"
#include "eeprom.h"
#include "qESC.h"


static int packet_drops = 0;
mavlink_message_t msg;
mavlink_status_t status;

xSemaphoreHandle DataSmphr;
xTaskHandle paramListHandle;

uint8_t rx_led = 0;
uint16_t m_parameter_i = ONBOARD_PARAM_COUNT; //esto es para que no mande al principio, solo en request

void UART_Rx_Handler(uint8_t * buff, size_t sz);


void ParameterSend(void * p){

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	while(1)
		if (m_parameter_i < ONBOARD_PARAM_COUNT)
		{
			mavlink_msg_param_value_pack( 	quadrotor.mavlink_system.sysid,
											quadrotor.mavlink_system.compid,
											&msg,
											(int8_t*) global_data.param_name[m_parameter_i],
											global_data.param[m_parameter_i],
											MAVLINK_TYPE_FLOAT,
											ONBOARD_PARAM_COUNT,
											m_parameter_i
										);

			len = mavlink_msg_to_send_buffer(buf, &msg);
			qUART_Send(UART_GROUNDCOMM,buf,len);

			m_parameter_i++;
			vTaskDelay(50/portTICK_RATE_MS);
		}else{
			vTaskSuspend(NULL);
		}
}

void ParameterSet(mavlink_message_t * msg){

	mavlink_message_t msg_out;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	mavlink_param_set_t set;
	mavlink_msg_param_set_decode(msg, &set);
	uint16_t i,j;

	// Check if this message is for this system
	if (((uint8_t) set.target_system == (uint8_t) quadrotor.mavlink_system.sysid)
	 && ((uint8_t) set.target_component == (uint8_t) quadrotor.mavlink_system.compid))
	{
		char* key = (char*) set.param_id;

		for ( i = 0; i < ONBOARD_PARAM_COUNT; i++)
		{
			bool match = TRUE;
			for ( j = 0; j < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN; j++)
			{
				// Compare
				if (((char) (global_data.param_name[i][j]))
						!= (char) (key[j]))
				{
					match = FALSE;
				}

				// End matching if null termination is reached
				if (((char) global_data.param_name[i][j]) == '\0')
				{
					break;
				}
			}

			// Check if matched
			if (match)
			{
				// Only write and emit changes if there is actually a difference
				// AND only write if new value is NOT "not-a-number"
				// AND is NOT infinity
				if (global_data.param[i] != set.param_value
						//&& !isnan(set.param_value)
						//&& !isinf(set.param_value) && set.param_type == MAVLINK_TYPE_FLOAT)
						)
				{
					global_data.param[i] = set.param_value;
					vPortEnterCritical();
					mavlink_msg_param_value_pack( 	quadrotor.mavlink_system.sysid,
							quadrotor.mavlink_system.compid,
							&msg_out,
							(int8_t*) global_data.param_name[i],
							global_data.param[i],
							MAVLINK_TYPE_FLOAT,
							ONBOARD_PARAM_COUNT,
							m_parameter_i
					);
					vPortExitCritical();
					len = mavlink_msg_to_send_buffer(buf, &msg_out);
					qUART_Send(UART_GROUNDCOMM,buf,len);



				}
			}
		}
	}


}

void Communications(void * pvParameters){

	vSemaphoreCreateBinary(DataSmphr);

    if (DataSmphr==NULL){
    	halt();
    }

    while (qUARTStatus[UART_GROUNDCOMM]!=DEVICE_READY);

    qUART_Register_RBR_Callback(UART_GROUNDCOMM, UART_Rx_Handler);
    qUART_EnableRx(UART_GROUNDCOMM);

    global_data_reset_param_defaults();
    xTaskCreate( ParameterSend, "PARAMS", 300, NULL, tskIDLE_PRIORITY+1,  &paramListHandle  );

	for (;;){
		if (pdTRUE == xSemaphoreTake(DataSmphr,1500/portTICK_RATE_MS)){
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
						quadrotor.mavlink_system.nav_mode = NAV_TAKEOFF;
						quadrotor.sv.setpoint[ALTITUDE] = 0.7;
						break;
					case MAV_CMD_NAV_LAND:
						quadrotor.mavlink_system.nav_mode = NAV_LANDING;
						quadrotor.sv.setpoint[ALTITUDE] = 0.7; // Bug horrible!
						break;
					case MAV_CMD_COMPONENT_ARM_DISARM:
						if (mavlink_msg_command_long_get_param1(&msg)==1){
						}else{
						}
						break;
					case MAV_CMD_PREFLIGHT_STORAGE:
						if (mavlink_msg_command_long_get_param1(&msg)==1){
							// Write parameters from flash
							vPortEnterCritical();
							eeprom_write(EEPROM_ADDRESS,&global_data.param[0],0x0000,sizeof(global_data.param));
							vPortExitCritical();
						}else{
							// Read parameters from flash
							vPortEnterCritical();
							eeprom_read(EEPROM_ADDRESS,&global_data.param[0],0x0000,sizeof(global_data.param));
							vPortExitCritical();
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
							quadrotor.mode = ESC_STANDBY;
						}else{
							quadrotor.mode = ESC_ARMED;
							/*
							quadrotor.sv.setpoint[ALTITUDE] = ((quadrotor.mavlink_control.x+1000.0)/2000.0)*500.0;
							qESC_SetOutput(MOTOR1,quadrotor.sv.setpoint[ALTITUDE]);
							qESC_SetOutput(MOTOR2,quadrotor.sv.setpoint[ALTITUDE]);
							qESC_SetOutput(MOTOR3,quadrotor.sv.setpoint[ALTITUDE]);
							qESC_SetOutput(MOTOR4,quadrotor.sv.setpoint[ALTITUDE]);
							*/
						}
						break;
					case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
						m_parameter_i = 0;
						vTaskResume(paramListHandle);
						break;

					case MAVLINK_MSG_ID_PARAM_SET:
						ParameterSet(&msg);
						break;

					default:
						//Do nothing
						break;
			}

		}else{
			// Timeout to get a new joystick commands, values to 0
			quadrotor.mode = ESC_STANDBY;
			qESC_SetOutput(MOTOR1,0);
			qESC_SetOutput(MOTOR2,0);
			qESC_SetOutput(MOTOR3,0);
			qESC_SetOutput(MOTOR4,0);
		}
	}
}


void UART_Rx_Handler(uint8_t * buff, size_t sz){
	uint32_t i;
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;


	for (i=0;i<sz;i++){
		if(mavlink_parse_char(MAVLINK_COMM_0, *(buff+i), &msg, &status)) {
			xSemaphoreGiveFromISR(DataSmphr,&xHigherPriorityTaskWoken);
		}
	}

	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken);
}


