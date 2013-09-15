/*
 * telemetry.c
 *
 *  Created on: 13/09/2013
 *      Author: alan
 */

#include "quadrotor.h"
#include "board.h"
#include "DebugConsole.h"
#include "types.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mavlink.h"
#include "mavlink_bridge.h"
#include "qUART.h"

void Telemetry(void * p){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();

	while(1){
		if (qUARTStatus[UART_GROUNDCOMM]==DEVICE_READY){
			mavlink_msg_attitude_pack(	quadrotor.mavlink_system.sysid,
										quadrotor.mavlink_system.compid,
										&msg,
										xTaskGetTickCount()/portTICK_RATE_MS,
										quadrotor.sv.attitude[0],
										quadrotor.sv.attitude[1],
										quadrotor.sv.attitude[2],
										quadrotor.sv.rate[0],
										quadrotor.sv.rate[1],
										quadrotor.sv.rate[2]);

			len = mavlink_msg_to_send_buffer(buf, &msg);
			qUART_Send(UART_GROUNDCOMM,buf,len);

			mavlink_msg_vfr_hud_pack(quadrotor.mavlink_system.sysid,
									quadrotor.mavlink_system.compid,
									&msg,
									0.0, // Airspeed
									0.0, // Groundspeed
									250, //TODO: Change to realheading
									0,   //TODO: Change to real throtle
									quadrotor.sv.altitude,
									0.0  //TODO: Change to ascent rate
									);

			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			qUART_Send(UART_GROUNDCOMM,buf,len);


#if USE_BAROMETER
			mavlink_msg_scaled_pressure_pack(quadrotor.mavlink_system.sysid,
											quadrotor.mavlink_system.compid,
											&msg,
											xTaskGetTickCount()/portTICK_RATE_MS,
											quadrotor.sv.current_pressure,
											quadrotor.sv.current_pressure-quadrotor.sv.floor_pressure,
											(int) quadrotor.sv.temperature * 100.0);

			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			qUART_Send(UART_GROUNDCOMM,buf,len);
#endif
		}
		vTaskDelayUntil( &xLastWakeTime, 20/portTICK_RATE_MS);
	}
}
