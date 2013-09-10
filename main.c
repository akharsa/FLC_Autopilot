/*
 * main.c
 *
 *  Created on: 08/09/2013
 *      Author: alan
 */

#include "qI2C.h"
#include "leds.h"
#include "board.h"
#include "DebugConsole.h"

#include "bmp085.h"
#include "eMPL/inv_mpu.h"
#include "lpc17xx_gpio.h"
#include "types.h"

#include "math.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mavlink.h"

#include "qUART.h"
#include "board.h"

#include "mavlink_bridge.h"

void Telemetry(void * p){
	float altitude, bmp_temp, pressure;
	float floor_pressure=0.0;
	//===========================================================
	// Barometer test
	BMP085_Init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();


	uint32_t i;
	for (i=0;i<100;i++){
		BMP085_GetTemperature();
		floor_pressure += BMP085_GetPressure()/100.0;
		vTaskDelay(10/portTICK_RATE_MS);
	}

	float alt = 0.0,c=0.1;

	while(1){
		bmp_temp = BMP085_GetTemperature();
		pressure = BMP085_GetPressure();
		alt =  c*BMP085_CalculateAltitude(floor_pressure, pressure) + (1-c)*alt;
		//altitude = BMP085_CalculateAltitude(floor_pressure, pressure);

		printf("%f \t\t %f \t\t %f\r\n", bmp_temp, pressure, alt);
		vTaskDelayUntil( &xLastWakeTime, 50/portTICK_RATE_MS);
	}

}


void MAVLink(void *p){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();

	mavlink_init(qUART_Send);

	while(1){
		//fwrite(buf, 1, len, fp);
		//qUART_Send(UART_GROUNDCOMM,buf,len);
		mavlink_heartbeat();
		mavlink_system_status();
		vTaskDelayUntil( &xLastWakeTime, 500/portTICK_RATE_MS);
	}

	return 0;
}


void AppMain(void){
	uint16_t i;
	printf("AutoPilot self test. \r\n");

	// =========================================================
	//	Leds Initialization
	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOn(leds[i]);
	}

	// =========================================================
	// I2C initialization
	debug("Initializing I2C interface...");
	if (qI2C_Init()!=SUCCESS){
		debug("[ERROR]\r\n");
		//halt();
	}
	debug("[OK]\r\n");

	// =========================================================
	// MPU6050 init

	debug("Initializing MPU6050 IMU...");
	if (mpu_init(NULL)==0){
		debug("[OK]\r\n");
	}else{
		debug("[ERROR]\r\n");
		//halt();
	}
	//===========================================================
	// I2C Scan

	uint8_t address;
	debug("Starting I2C scan...\r\n");
	address = 0;
	do {
		if (qI2C_Write(address,NULL,0x00,0)==SUCCESS){
			//debug("0x");
			debug_fmt("0x%x",address);
			debug(" Address found\r\n");
		}
	} while (address++ != 255);
	debug("I2C scan finished\r\n");


	//============================================================
	// UART
	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)!=RET_OK){
		//halt();
	}
	qUART_EnableTx(UART_GROUNDCOMM);

	//xTaskCreate( Telemetry, "TLM", 300, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( MAVLink, "MAVLINK", 300, NULL, tskIDLE_PRIORITY, NULL );
	vTaskStartScheduler();
	while(1);
}

