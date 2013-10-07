/*
 * halt.c
 *
 *  Created on: 13/09/2013
 *      Author: alan
 */

#include "DebugConsole.h"
#include "leds.h"
#include "board.h"


void halt(void){
	uint32_t i;

	for (i=0;i<TOTAL_LEDS;i++){
		qLed_Init(leds[i]);
		qLed_TurnOn(leds[i]);
	}

	debug("Halted!\n");

	for(;;){
		for (i=1;i<5;i++) qLed_TurnOn(leds[i]);
		for (i=0;i<99999;i++);
		for (i=1;i<5;i++) qLed_TurnOff(leds[i]);
		for (i=0;i<99999;i++);
	}
}

