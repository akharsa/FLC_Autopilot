/*
 * main.c
 *
 *  Created on: 08/09/2013
 *      Author: alan
 */


#include  "quadrotor.h"
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

#include "math.h"

void hardware_init(void *);
void MAVLink_Heartbeat(void *);
void Telemetry(void *);

void AppMain(void){


	quadrotor.mavlink_system.sysid = 20;
	quadrotor.mavlink_system.compid = 01;
	quadrotor.mavlink_system.type = MAV_TYPE_QUADROTOR;
	quadrotor.mavlink_system.state = MAV_STATE_BOOT;
	quadrotor.mavlink_system.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	quadrotor.mavlink_system.nav_mode = MAV_AUTOPILOT_GENERIC;

	xTaskCreate( hardware_init, "HW_INIT", 300, NULL, tskIDLE_PRIORITY+1, NULL );
	xTaskCreate( MAVLink_Heartbeat, "HEARTBEAT", 300, NULL, tskIDLE_PRIORITY+2, NULL );
	xTaskCreate( Telemetry, "TLM", 300, NULL, tskIDLE_PRIORITY+1, NULL );

	//xTaskCreate( MAVlink_Telemetry, "TLM", 300, NULL, tskIDLE_PRIORITY+2, NULL );
	//xTaskCreate( SensorCollector, "IMU", 300, NULL, tskIDLE_PRIORITY+1, NULL );
	vTaskStartScheduler();
	while(1);
}

