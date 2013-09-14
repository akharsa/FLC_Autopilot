/*
 * heartbeat.c
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

void MAVLink_Heartbeat(void *p){
	portTickType xLastWakeTime;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;
	uint16_t len;

	xLastWakeTime = xTaskGetTickCount ();

	while(1){
		if (qUARTStatus[UART_GROUNDCOMM]==DEVICE_READY){
			mavlink_msg_heartbeat_pack(	quadrotor.mavlink_system.sysid,
										quadrotor.mavlink_system.compid,
										&msg,
										quadrotor.mavlink_system.type,
										quadrotor.mavlink_system.nav_mode,
										quadrotor.mavlink_system.mode,
										0,
										quadrotor.mavlink_system.state);

			len = mavlink_msg_to_send_buffer(buf, &msg);
			qUART_Send(UART_GROUNDCOMM,buf,len);


			mavlink_msg_sys_status_pack(quadrotor.mavlink_system.sysid,
										quadrotor.mavlink_system.compid,
										&msg,
										SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE,
										SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE,
										SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE, //Sensors health
										950, //TODO: Change to CPU load  in 0.1%
										10700, //TODO: change to battery voltage in mV
										-1,	//Battery current not measured
										-1, //Battery SoC not measured
										0,0,0,0,0,0); //Error counts

			len = mavlink_msg_to_send_buffer(buf, &msg);
			qUART_Send(UART_GROUNDCOMM,buf,len);

			mavlink_msg_vfr_hud_pack(quadrotor.mavlink_system.sysid,
									quadrotor.mavlink_system.compid,
									&msg,
									0.0, // Airspeed
									0.0, // Groundspeed
									250, //TODO: Change to heading
									0,   //TODO: Change to throtle
									1.5, //TODO: Change to altitude
									0.0  //TODO: Change to ascent rate
									);

			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			qUART_Send(UART_GROUNDCOMM,buf,len);


		}

		vTaskDelayUntil( &xLastWakeTime, 1000/portTICK_RATE_MS);

		/*
		uint32_t sensors = SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE;
		uint32_t active_sensors = SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE;
		uint32_t sensors_health = SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE;
		mavlink_msg_sys_status_pack(mavlink_system.sysid,mavlink_system.compid,&msg,sensors,active_sensors,sensors_health,950,10700,12,-1,0,0,0,0,0,0);

		// Copy the message to the send buffer
		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
		(*sender_function)(UART_GROUNDCOMM,buf,len);
*/

	}
}


