/*
 * runtime_stat_timer.c
 *
 *  Created on: 15/09/2013
 *      Author: alan
 */


#include "lpc17xx_timer.h"
void vConfigureTimerForRunTimeStats(void){

	TIM_TIMERCFG_Type cfg;
	cfg.PrescaleOption = TIM_PRESCALE_USVAL;
	cfg.PrescaleValue = 1;
	TIM_Init(LPC_TIM1,TIM_TIMER_MODE,&cfg);
	TIM_Cmd(LPC_TIM1,ENABLE);
}
