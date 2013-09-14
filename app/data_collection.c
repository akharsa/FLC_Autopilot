/*
 * data_collection.c

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
#include "lpc17xx_gpio.h"

#include "math.h"
#include "bmp085.h"
#include "HMC5883L.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"


xSemaphoreHandle mpuSempahore;

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

void DataCollection(void *p){

	int16_t gyro[3];
	int16_t accel[3];
	int32_t quat[4];
	float scale;
	uint8_t more;
	float atti_buffer[3];
	int16_t sensors;
	float bmp_temp, pressure, alt=0.0, c;


	mpu_get_gyro_sens(&scale);
	vSemaphoreCreateBinary(mpuSempahore);
	xSemaphoreTake(mpuSempahore,0);
	NVIC_EnableIRQ(EINT3_IRQn);

	// ---- Barometer config ------------
	c = 0.1;

	while(1){

		// Wait here for MPU DMP interrupt at 200Hz
		xSemaphoreTake(mpuSempahore,portMAX_DELAY); //FIXME: instead of portMAX it would be nice to have a time out for errors

		portENTER_CRITICAL();
		dmp_read_fifo(gyro, accel, quat, NULL, &sensors, &more);
		portEXIT_CRITICAL();

		MPU6050_dmpGetEuler(atti_buffer,quat);

		quadrotor.sv.attitude[0] = atti_buffer[2];
		quadrotor.sv.attitude[1] = -atti_buffer[1];
		quadrotor.sv.attitude[2] = atti_buffer[0];
		quadrotor.sv.rate[0] = -atti_buffer[0]/scale;
		quadrotor.sv.rate[1] = atti_buffer[1]/scale;
		quadrotor.sv.rate[2] = -atti_buffer[2]/scale;

#if USE_BAROMETER
		//quadrotor.sv.temperature = BMP085_GetTemperature();
		//quadrotor.sv.current_pressure = BMP085_GetPressure();
		//quadrotor.sv.altitude =  c*BMP085_CalculateAltitude(quadrotor.sv.floor_pressure, quadrotor.sv.current_pressure) + (1-c)*quadrotor.sv.altitude;
#endif

	}

}


#if 0
void MAVlink_Telemetry(void * p){
	float altitude, bmp_temp, pressure;
	float floor_pressure=0.0;
	float alt = 0.0, c=0.1;
	uint32_t i;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();


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


		//HMC5883L_setMeasurementBias(0);
		//HMC5883L_getHeading(&mag[0],&mag[1],&mag[2]);
		//printf("%i \t %i \t %i\n",mag[0],mag[1],mag[2]);

		mvalink_send_telemetry(pressure,floor_pressure,bmp_temp);
		mavlink_send_hud(alt);
		vTaskDelayUntil( &xLastWakeTime, 500/portTICK_RATE_MS);
	}


}


#endif

