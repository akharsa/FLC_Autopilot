/*
 * mavlink_bridge.h
 *
 *  Created on: Sep 10, 2013
 *      Author: Alan
 */

#ifndef MAVLINK_BRIDGE_H_
#define MAVLINK_BRIDGE_H_

/* MAVLink adapter header */
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"
#include "qUART.h"
#include "FreeRTOS.h"

#define SENSOR_GYRO	(1<<0)
#define SENSOR_ACC	(1<<1)
#define SENSOR_MAG	(1<<2)
#define SENSOR_ABS_PRESSURE	(1<<3)
#define SENSOR_DIFF_PRESSURE (1<<4)
#define SENSOR_GPS	(1<<5)
#define SENSOR_OPTICAL_FLOW	(1<<6)
#define SENSOR_COMPUTER_VISION_POSITION	(1<<7)
#define SENSOR_LASER_POSITION	(1<<8)
#define SENSOR_VICON	(1<<9)


static mavlink_system_t mavlink_system;
static uint32_t (*sender_function)(uint8_t id, uint8_t * buff, size_t size);

void mavlink_init(uint32_t (*fp)(uint8_t id, uint8_t * buff, size_t size)){
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = 54;    		 		 ///< The component sending the message is the IMU, it could be also a Linux process
	mavlink_system.type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
	sender_function = fp;
}

void mavlink_heartbeat(){

	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t system_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; ///< Booting up
	uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, autopilot_type, system_mode, custom_mode, system_state);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	(*sender_function)(UART_GROUNDCOMM,buf,len);
}

void mavlink_system_status(){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint32_t sensors = SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE;
	uint32_t active_sensors = SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE;
	uint32_t sensors_health = SENSOR_GYRO | SENSOR_ACC | SENSOR_MAG | SENSOR_ABS_PRESSURE;
	mavlink_msg_sys_status_pack(mavlink_system.sysid,mavlink_system.compid,&msg,sensors,active_sensors,sensors_health,950,10700,12,-1,0,0,0,0,0,0);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	(*sender_function)(UART_GROUNDCOMM,buf,len);
}

void mvalink_send_telemetry(float absolute_pressure, float differential_pressure, float temperature){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_scaled_pressure_pack(mavlink_system.sysid,mavlink_system.compid,&msg,xTaskGetTickCount(),absolute_pressure,differential_pressure,temperature);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	(*sender_function)(UART_GROUNDCOMM,buf,len);
}



#endif /* MAVLINK_BRIDGE_H_ */
