/*
 * distance.c
 *
 *  Created on: 14/09/2013
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
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"

TIM_TIMERCFG_Type TIM_ConfigStruct;
TIM_MATCHCFG_Type TIM_MatchConfigStruct ;
TIM_CAPTURECFG_Type TIM_CaptureConfigStruct;
volatile uint8_t captured = 0;

void TIMER3_IRQHandler(void)
{
	if (TIM_GetIntCaptureStatus(LPC_TIM3,0))
	{
		TIM_ClearIntCapturePending(LPC_TIM3,0);
		quadrotor.sv.altitude = TIM_GetCaptureValue(LPC_TIM3,0)/58.0;
		captured = 1;
		//_DBH32(TIM_GetCaptureValue(LPC_TIM1,0));_DBG_("");
	}
}

void Distance(void *p){

	PINSEL_CFG_Type PinCfg;
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
	TIM_CaptureConfigStruct.RisingEdge = DISABLE;
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

	uint32_t i;

	while(1){
		PinCfg.Funcnum = 0;
		PINSEL_ConfigPin(&PinCfg);
		GPIO_SetDir(0,(1<<23),1); // Output
		GPIO_SetValue(0,(1<<23));
		for (i=0;i<999999;i++);
		GPIO_ClearValue(0,(1<<23));

		GPIO_SetDir(0,(1<<23),0); // Input
		while ((GPIO_ReadValue(0)&(1<<23)) == 0);
		PinCfg.Funcnum = 3;
		PINSEL_ConfigPin(&PinCfg);
		TIM_ResetCounter(LPC_TIM3);
		TIM_Cmd(LPC_TIM3,ENABLE);

		while(captured==0);
		captured = 0;
		vTaskDelay(200/portTICK_RATE_MS);
	}
}
