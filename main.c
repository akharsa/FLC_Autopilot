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
#include "queue.h"
#include "semphr.h"


#include "mavlink.h"

#include "qUART.h"
#include "board.h"

#include "mavlink_bridge.h"

#include "HMC5883L.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"

uint8_t MPU6050_dmpGetEuler(float *euler, int32_t q[]) {

	float q1[4];
	uint8_t i;

	for(i = 0; i < 4; i++ ) {
		q1[i] = ((float) (q[i]>>16)) / 16384.0f;
	}

	euler[0] = atan2(2*q1[1]*q1[2] - 2*q1[0]*q1[3], 2*q1[0]*q1[0] + 2*q1[1]*q1[1] - 1);
	euler[1] = -asin(2*q1[1]*q1[3] + 2*q1[0]*q1[2]);
	euler[2] = atan2(2*q1[2]*q1[3] - 2*q1[0]*q1[1], 2*q1[0]*q1[0] + 2*q1[3]*q1[3] - 1);

	return 0;
}

xSemaphoreHandle mpuSempahore;

void SensorCollector(void *p){
	int16_t gyro[3];
	int16_t accel[3];
	int32_t quat[4];
	float scale;
	uint8_t more;
	float atti_buffer[3];
	int16_t sensors;


	// MPU config
	mpu_get_gyro_sens(&scale);
	vSemaphoreCreateBinary(mpuSempahore);

	xSemaphoreTake(mpuSempahore,0);
	NVIC_EnableIRQ(EINT3_IRQn);


	while(1){
	// Wait here for MPU DMP interrupt at 200Hz
		xSemaphoreTake(mpuSempahore,portMAX_DELAY); //FIXME: instead of portMAX it would be nice to have a time out for errors

		portENTER_CRITICAL();
		dmp_read_fifo(gyro, accel, quat, NULL, &sensors, &more);
		portEXIT_CRITICAL();
		MPU6050_dmpGetEuler(atti_buffer,quat);

		mavlink_send_attitude(  atti_buffer[2],\
								-atti_buffer[1], \
								atti_buffer[0], \
								-gyro[0]/scale, \
								gyro[1]/scale, \
								-gyro[2]/scale );
	}

}


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

	//int16_t mag[3];

	//HMC5883L_initialize();
	//if (HMC5883L_testConnection()!=TRUE){
	//}


	while(1){
		bmp_temp = BMP085_GetTemperature();
		pressure = BMP085_GetPressure();
		alt =  c*BMP085_CalculateAltitude(floor_pressure, pressure) + (1-c)*alt;

		//HMC5883L_setMeasurementBias(0);
		//HMC5883L_getHeading(&mag[0],&mag[1],&mag[2]);
		//printf("%i \t %i \t %i\n",mag[0],mag[1],mag[2]);

		mvalink_send_telemetry(pressure,floor_pressure,bmp_temp);
		mavlink_send_hud(alt);
		vTaskDelayUntil( &xLastWakeTime, 500/portTICK_RATE_MS);
	}


}


// Hardware IRQ for the MPU6050 DMP algorithm.
void EINT3_IRQHandler(void)
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

	if(GPIO_GetIntStatus(0, 4, 1))
	{
		GPIO_ClearInt(0,(1<<4));
		if (mpuSempahore!=NULL){
			xSemaphoreGiveFromISR(mpuSempahore,&xHigherPriorityTaskWoken);
		}
	}
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

xSemaphoreHandle mpuSempahore;


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
	//============================================================
	// UART
	if (qUART_Init(UART_GROUNDCOMM,57600,8,QUART_PARITY_NONE,1)!=RET_OK){
		//halt();
	}
	qUART_EnableTx(UART_GROUNDCOMM);


	debug("AutoPilot self test. \r\n");

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

#if 1
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

	// GPIO0.4 as input with interrupt
	GPIO_SetDir(0,(1<<4),0);
	GPIO_IntCmd(0,(1<<4),1);
	GPIO_ClearInt(0,(1<<4));
	NVIC_SetPriority(EINT3_IRQn, 6);
	NVIC_EnableIRQ(EINT3_IRQn);

	debug("MPU Init ready");
#endif
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



	mavlink_init(qUART_Send);


	xTaskCreate( MAVlink_Telemetry, "TLM", 300, NULL, tskIDLE_PRIORITY+2, NULL );
	xTaskCreate( MAVLink_Heartbeat, "HEARTBEAT", 300, NULL, tskIDLE_PRIORITY+2, NULL );
	xTaskCreate( SensorCollector, "IMU", 300, NULL, tskIDLE_PRIORITY+1, NULL );
	vTaskStartScheduler();
	while(1);
}

