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
#include "qWDT.h"
#include "qPWM.h"
#include "tasks.h"

void Communications(void *);
void AppMain(void){
	portBASE_TYPE ret;

	if (qWDT_GetResetSource()==RESET_EXTERNAL){
		//First boot
	}else{
		//Watchdog Boot
		//halt();
	}
	quadrotor.mavlink_system.sysid = 20;
	quadrotor.mavlink_system.compid = 01;
	quadrotor.mavlink_system.type = MAV_TYPE_QUADROTOR;

	quadrotor.mavlink_system.state = MAV_STATE_BOOT;
	quadrotor.mavlink_system.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	quadrotor.mavlink_system.nav_mode = NAV_ATTI;

	ret = xTaskCreate( hardware_init, "HW_INIT", 150, NULL, tskIDLE_PRIORITY+3, NULL ); //STACK OK,
	ret = xTaskCreate( MAVLink_Heartbeat, "HEARTBEAT", 300, NULL, tskIDLE_PRIORITY+2, NULL ); //STACK OK

	vTaskStartScheduler();
	while(1);
}

void WDT_IRQHandler(void){
	uint32_t i;
	qWDT_GetResetSource();
	//quadrotor.mode = ESC_STANDBY;
	//quadrotor.mavlink_system.state = MAV_STATE_STANDBY;
	quadrotor.mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	qESC_SetOutput(MOTOR1,0);
	qESC_SetOutput(MOTOR2,0);
	qESC_SetOutput(MOTOR3,0);
	qESC_SetOutput(MOTOR4,0);
	for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
	for (i=0;i<999999;i++);
	for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
	for (i=0;i<999999;i++);

}
