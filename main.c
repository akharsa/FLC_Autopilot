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

#include "HMC5883L.h"


void MAVlink_Telemetry(void * p){
	float altitude, bmp_temp, pressure;
	float floor_pressure=0.0;
	float alt = 0.0, c=0.1;
	uint32_t i;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();

	BMP085_Init();

	for (i=0;i<100;i++){
		BMP085_GetTemperature();
		floor_pressure += BMP085_GetPressure()/100.0;
		vTaskDelay(10/portTICK_RATE_MS);
	}

	int16_t mag[3];

	HMC5883L_initialize();
	if (HMC5883L_testConnection()!=TRUE){
	}


	uint8_t gain = HMC5883L_getGain();

	while(1){
		bmp_temp = BMP085_GetTemperature();
		pressure = BMP085_GetPressure();
		alt =  c*BMP085_CalculateAltitude(floor_pressure, pressure) + (1-c)*alt;

		HMC5883L_setMeasurementBias(0);
		HMC5883L_getHeading(&mag[0],&mag[1],&mag[2]);
		//printf("%i \t %i \t %i\n",mag[0],mag[1],mag[2]);

		mvalink_send_telemetry(pressure,floor_pressure,bmp_temp);

		vTaskDelayUntil( &xLastWakeTime, 500/portTICK_RATE_MS);
	}

}


void MAVLink_Heartbeat(void *p){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();

	while(1){
		mavlink_heartbeat();
		mavlink_system_status();
		vTaskDelayUntil( &xLastWakeTime, 1000/portTICK_RATE_MS);
	}
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

	mavlink_init(qUART_Send);


	xTaskCreate( MAVlink_Telemetry, "TLM", 300, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( MAVLink_Heartbeat, "HEARTBEAT", 300, NULL, tskIDLE_PRIORITY, NULL );
	vTaskStartScheduler();
	while(1);
}

