/*
 * hardware_init.c
 *
 *  Created on: 13/09/2013
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
#include "qI2C.h"
#include "lpc17xx_gpio.h"

#include "leds.h"
#include "bmp085.h"
#include "HMC5883L.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"
#include "ultrasonic_sensor.h"
#include "math.h"
#include "qESC.h"
#include "qAnalog.h"
#include "tasks.h"

xTaskHandle beacon_hnd;


void hardware_init(void * p){
	uint16_t i;
	portBASE_TYPE ret;

	// =========================================================
	// Early init will turn the motors off for safety if a reset occurs
	qESC_Init();
	qESC_InitChannel(MOTOR1);
	qESC_InitChannel(MOTOR2);
	qESC_InitChannel(MOTOR3);
	qESC_InitChannel(MOTOR4);

	vTaskDelay(5000/portTICK_RATE_MS);
	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)!=RET_OK){
		halt();
	}
	qUART_EnableTx(UART_GROUNDCOMM);

	debug("Initializing hardware \r\n");

	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOn(leds[i]);
	}

	debug("Initializing I2C interface...");
	if (qI2C_Init()!=SUCCESS){
		debug("[ERROR]\r\n");
		halt();
	}
	debug("[OK]\r\n");

	debug("Downloading MPU6050 eMPL firmware...");
	if (mpu_init(NULL)==0){
		debug("[OK]\r\n");
	}else{
		debug("[ERROR]\r\n");
		halt();
	}

	debug("Configuring MPU6050 IMU...");
	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	/* Push both gyro and accel data into the FIFO. */
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(200);
	dmp_load_motion_driver_firmware();
	mpu_set_gyro_fsr(2000);
	mpu_set_accel_fsr(2);
	mpu_set_lpf(150);
	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	dmp_set_fifo_rate(200);
	dmp_enable_gyro_cal(1);
	mpu_set_dmp_state(1);


	// TODO: estaria bueno que esto este adentro del driver (por ahi)
	// GPIO0.4 as input with interrupt
	GPIO_SetDir(0,(1<<4),0);
	GPIO_IntCmd(0,(1<<4),1);
	GPIO_ClearInt(0,(1<<4));
	NVIC_SetPriority(EINT3_IRQn, 6);
	NVIC_DisableIRQ(EINT3_IRQn);

	debug("[OK]\r\n");

#if 0
	//===========================================================
	// I2C Scan

	uint8_t address;
	debug("Starting I2C scan...\r\n");
	address = 0;
	do {
		if (qI2C_Write(address,NULL,0x00,0)==SUCCESS){
			//debug("0x");
			//debug_fmt("0x%x",address);
			debug(" Address found\r\n");
		}
	} while (address++ != 255);
	debug("I2C scan finished\r\n");
#endif

	BMP085_Init();
	RangeFinder_Init();

	//=========================================================================
	quadrotor.mavlink_system.state = MAV_STATE_CALIBRATING;

	quadrotor.sv.floor_pressure = 0.0;
/*
	for (i=0;i<100;i++){
		BMP085_GetTemperature();
		quadrotor.sv.floor_pressure += BMP085_GetPressure()/100.0;
		vTaskDelay(10/portTICK_RATE_MS);
	}
*/
	for (i=0;i<3;i++){
		quadrotor.rateController[i].AntiWindup = ENABLED;
		quadrotor.rateController[i].Bumpless = ENABLED;
		quadrotor.rateController[i].Mode = AUTOMATIC;
		quadrotor.rateController[i].OutputMax = 1.0;
		quadrotor.rateController[i].OutputMin = -1.0;
		quadrotor.rateController[i].Ts = 0.005;
		quadrotor.rateController[i].b = 1.0;
		quadrotor.rateController[i].c = 1.0;
		qPID_Init(&quadrotor.rateController[i]);
	}

	for (i=0;i<3;i++){
		quadrotor.attiController[i].AntiWindup = ENABLED;
		quadrotor.attiController[i].Bumpless = ENABLED;
		quadrotor.attiController[i].Mode = AUTOMATIC;
		quadrotor.attiController[i].OutputMax = 250.0;
		quadrotor.attiController[i].OutputMin = -250.0;
		quadrotor.attiController[i].Ts = 0.005;
		quadrotor.attiController[i].b = 1.0;
		quadrotor.attiController[i].c = 1.0;
		qPID_Init(&quadrotor.attiController[i]);
	}


	quadrotor.altitudeController.AntiWindup = ENABLED;
	quadrotor.altitudeController.Bumpless = ENABLED;
	quadrotor.altitudeController.Mode = AUTOMATIC;
	quadrotor.altitudeController.OutputMax = 1.0;
	quadrotor.altitudeController.OutputMin = 0.0;
	quadrotor.altitudeController.Ts = 0.025;
	quadrotor.altitudeController.b = 1.0;
	quadrotor.altitudeController.c = 0.0;
	qPID_Init(&quadrotor.altitudeController);

	quadrotor.rateController[ROLL].K = 0.01;
	quadrotor.rateController[ROLL].Ti = 1/0.1;
	quadrotor.rateController[ROLL].Td = 0.0;
	quadrotor.rateController[ROLL].Nd = 5.0;

	quadrotor.rateController[PITCH].K = 0.01;
	quadrotor.rateController[PITCH].Ti = 1/0.1;
	quadrotor.rateController[PITCH].Td = 0.0;
	quadrotor.rateController[PITCH].Nd = 5.0;
	// --------------------------------------------------------
	quadrotor.attiController[ROLL].K = 1.0;
	quadrotor.attiController[ROLL].Ti = 1/0.02;
	quadrotor.attiController[ROLL].Td = 0.0;
	quadrotor.attiController[ROLL].Nd = 4.0;

	quadrotor.attiController[PITCH].K = 1.0;
	quadrotor.attiController[PITCH].Ti = 1/0.02;
	quadrotor.attiController[PITCH].Td = 0.0;
	quadrotor.attiController[PITCH].Nd = 4.0;
	// --------------------------------------------------------
	quadrotor.rateController[YAW].K = 0.05;
	quadrotor.rateController[YAW].Ti = 1/0.1;
	quadrotor.rateController[YAW].Td = 0.000;
	quadrotor.rateController[YAW].Nd = 5;

	quadrotor.attiController[YAW].K = 5;
	quadrotor.attiController[YAW].Ti = 1/0.05;
	quadrotor.attiController[YAW].Td = 0.0;
	quadrotor.attiController[YAW].Nd = 4;
	// --------------------------------------------------------
	quadrotor.altitudeController.K = 0.85;
	quadrotor.altitudeController.Ti = 1/1.4;
	quadrotor.altitudeController.Td = 0.3;
	quadrotor.altitudeController.Nd = 10;

	qAnalog_Init();
	qAnalog_InitPin(TEMPERATURE_ANALOG);
	qAnalog_InitPin(VOLTAGE_ANALOG);

	//=========================================================================

	quadrotor.mavlink_system.state = MAV_STATE_ACTIVE;
	quadrotor.mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	quadrotor.mode = ESC_STANDBY;
	quadrotor.mavlink_system.nav_mode = NAV_ATTI;

	MAVLink_parameters_setup();

	//uint32_t stack_free = uxTaskGetStackHighWaterMark(NULL);
	ret = xTaskCreate( DataCollection, "DATCOL", 500, NULL, tskIDLE_PRIORITY+3, NULL );
	ret = xTaskCreate( Telemetry, "TLM", 300, NULL, tskIDLE_PRIORITY+1, NULL );
	ret = xTaskCreate( Communications, "COMMS", 300, NULL, tskIDLE_PRIORITY+2, NULL );
	ret = xTaskCreate( beacon, "BEACON", 30, NULL, tskIDLE_PRIORITY+1, NULL); // STACK OK

	vTaskDelete(NULL);


}

