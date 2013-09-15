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
#include "ultrasonic_sensor.h"
#include "qESC.h"

xSemaphoreHandle mpuSempahore;
uint8_t prescaler = 0;

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


float map(long x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

	//===========================================
	prescaler = 10;
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

		if (prescaler-- == 0){
			RangeFinder_getDistance();
			prescaler = 10;
		}

		//quadrotor.sv.setpoint[ALTITUDE] = map((quadrotor.joystick.left_pad.y>127)?127:255-quadrotor.joystick.left_pad.y,127,255,0.0,1.0);
		quadrotor.sv.setpoint[ALTITUDE] = map((quadrotor.mavlink_control.x < 100)?0:quadrotor.mavlink_control.x,0,1000,0.0,1.0);
		//quadrotor.sv.setpoint[ROLL] = map(quadrotor.joystick.right_pad.x,0,255,-40.0,40.0);
		//quadrotor.sv.setpoint[PITCH] = map(quadrotor.joystick.right_pad.y,0,255,-40.0,40.0);
		//quadrotor.sv.setpoint[YAW] = map(quadrotor.joystick.left_pad.x,0,255,-180.0,180.0);

		quadrotor.sv.motorOutput[0] = K_Z*quadrotor.sv.setpoint[ALTITUDE];
		quadrotor.sv.motorOutput[1] = K_Z*quadrotor.sv.setpoint[ALTITUDE];
		quadrotor.sv.motorOutput[2] = K_Z*quadrotor.sv.setpoint[ALTITUDE];
		quadrotor.sv.motorOutput[3] = K_Z*quadrotor.sv.setpoint[ALTITUDE];

		if ((quadrotor.mavlink_system.mode & MAV_MODE_FLAG_SAFETY_ARMED) == 0){
			qESC_SetOutput(MOTOR1,0);
			qESC_SetOutput(MOTOR2,0);
			qESC_SetOutput(MOTOR3,0);
			qESC_SetOutput(MOTOR4,0);
		}else{
			// Motor command
			qESC_SetOutput(MOTOR1,quadrotor.sv.motorOutput[0]);
			qESC_SetOutput(MOTOR2,quadrotor.sv.motorOutput[1]);
			qESC_SetOutput(MOTOR3,quadrotor.sv.motorOutput[2]);
			qESC_SetOutput(MOTOR4,quadrotor.sv.motorOutput[3]);
		}
	}

}

