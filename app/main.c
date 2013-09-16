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

#include "tasks.h"

void Communications(void *);
void AppMain(void){
	portBASE_TYPE ret;

	quadrotor.mavlink_system.sysid = 20;
	quadrotor.mavlink_system.compid = 01;
	quadrotor.mavlink_system.type = MAV_TYPE_QUADROTOR;
	quadrotor.mavlink_system.state = MAV_STATE_BOOT;
	quadrotor.mavlink_system.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	quadrotor.mavlink_system.nav_mode = MAV_AUTOPILOT_GENERIC;

	ret = xTaskCreate( hardware_init, "HW_INIT", 150, NULL, tskIDLE_PRIORITY+3, NULL ); //STACK OK,


	vTaskStartScheduler();
	while(1);
}

