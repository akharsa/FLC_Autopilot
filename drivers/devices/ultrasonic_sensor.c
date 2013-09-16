/*
 * ultrasonic_sensor.c
 *
 *  Created on: 15/09/2013
 *      Author: alan
 */

#include "ultrasonic_sensor.h"

#include "quadrotor.h"
#include "DebugConsole.h"
#include "board.h"
#include "types.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mavlink.h"

#include "qUART.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"

#include "math.h"

TIM_TIMERCFG_Type TIM_ConfigStruct;
TIM_MATCHCFG_Type TIM_MatchConfigStruct ;
TIM_CAPTURECFG_Type TIM_CaptureConfigStruct;
PINSEL_CFG_Type PinCfg;

float c = 1.0 ;

void TIMER3_IRQHandler(void)
{
	uint32_t capture;
	float distance;

	if (TIM_GetIntCaptureStatus(LPC_TIM3,0))
	{
		TIM_ClearIntCapturePending(LPC_TIM3,0);

		if ((GPIO_ReadValue(0)&(1<<23)) != 0){
			//TIM_ResetCounter(LPC_TIM3);
		}else{
			capture = TIM_GetCaptureValue(LPC_TIM3,0);
			if (capture>35000){
				//quadrotor.sv.altitude = -100.0;
				// I think it's better to keep the last good value instead
			}else{
				distance = (capture/58.0)/100.0;
				//distance = floorf(distance * 100 + 0.5) / 100;
				quadrotor.sv.altitude = c*distance + (1-c)*quadrotor.sv.altitude;
			}
			TIM_Cmd(LPC_TIM3,DISABLE);
		}
	}
}

void RangeFinder_Init(void){

		/*
		 * P0.23, CAP3[0] (func 3)
		 */
		// ----- Distance sensor ------------
		GPIO_SetDir(0,(1<<23),1); // Output
		GPIO_ClearValue(0,(1<<23));

		PinCfg.Funcnum = 3;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Portnum = 0;
		PinCfg.Pinnum = 23;


		// Initialize timer 0, prescale count time of 1000000uS = 1S
		TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
		TIM_ConfigStruct.PrescaleValue	= 1;

		// use channel 0, CAPn.0
		TIM_CaptureConfigStruct.CaptureChannel = 0;
		// Enable capture on CAPn.0 rising edge
		TIM_CaptureConfigStruct.RisingEdge = ENABLE;
		// Enable capture on CAPn.0 falling edge
		TIM_CaptureConfigStruct.FallingEdge = ENABLE;
		// Generate capture interrupt
		TIM_CaptureConfigStruct.IntOnCaption = ENABLE;

		// Set configuration for Tim_config and Tim_MatchConfig
		TIM_Init(LPC_TIM3, TIM_TIMER_MODE,&TIM_ConfigStruct);
		TIM_ConfigCapture(LPC_TIM3, &TIM_CaptureConfigStruct);
		TIM_ResetCounter(LPC_TIM3);


		/* preemption = 1, sub-priority = 1 */
		NVIC_SetPriority(TIMER3_IRQn, ((0x01<<3)|0x01));
		/* Enable interrupt for timer 0 */
		NVIC_EnableIRQ(TIMER3_IRQn);

}

void RangeFinder_getDistance(void){

	uint32_t i;
	// Configure as output and fire
	PinCfg.Funcnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0,(1<<23),1); // Output

	GPIO_SetValue(0,(1<<23));
	for (i=0;i<9999;i++);
	GPIO_ClearValue(0,(1<<23));

	// Configure as timer input and leave. The rest of the processing
	// is done by the timer interrupt.
	GPIO_SetDir(0,(1<<23),0); // Input

	PinCfg.Funcnum = 3;
	PINSEL_ConfigPin(&PinCfg);
	TIM_ResetCounter(LPC_TIM3);
	TIM_Cmd(LPC_TIM3,ENABLE);
}


