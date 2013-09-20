/*
 * parameters.c
 *
 *  Created on: 14/09/2013
 *      Author: alan
 */

#include  "quadrotor.h"
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

#include "math.h"
#include "parameters.h"


/**
 * @brief reset all parameters to default
 * @warning DO NOT USE THIS IN FLIGHT!
 */
inline void MAVLink_parameters_setup(void)
{
	strcpy(global_data.param_name[RATE_ROLL_K],  "RATE_ROLL_K");
	global_data.param[RATE_ROLL_K] = &quadrotor.rateController[ROLL].K;
	strcpy(global_data.param_name[RATE_ROLL_TI], "RATE_ROLL_TI");
	global_data.param[RATE_ROLL_TI] = &quadrotor.rateController[ROLL].Ti;
	strcpy(global_data.param_name[RATE_ROLL_TD], "RATE_ROLL_TD");
	global_data.param[RATE_ROLL_TD] = &quadrotor.rateController[ROLL].Td;
	strcpy(global_data.param_name[RATE_ROLL_ND], "RATE_ROLL_ND");
	global_data.param[RATE_ROLL_ND] = &quadrotor.rateController[ROLL].Nd;

	strcpy(global_data.param_name[RATE_PITCH_K],  "RATE_PITCH_K");
	global_data.param[RATE_PITCH_K] = &quadrotor.rateController[PITCH].K;
	strcpy(global_data.param_name[RATE_PITCH_TI], "RATE_PITCH_TI");
	global_data.param[RATE_PITCH_TI] = &quadrotor.rateController[PITCH].Ti;
	strcpy(global_data.param_name[RATE_PITCH_TD], "RATE_PITCH_TD");
	global_data.param[RATE_PITCH_TD] = &quadrotor.rateController[PITCH].Td;
	strcpy(global_data.param_name[RATE_PITCH_ND], "RATE_PITCH_ND");
	global_data.param[RATE_PITCH_ND] = &quadrotor.rateController[PITCH].Nd;

	strcpy(global_data.param_name[RATE_YAW_K], "RATE_YAW_K");
	global_data.param[RATE_YAW_K] = &quadrotor.rateController[YAW].K;
	strcpy(global_data.param_name[RATE_YAW_TI], "RATE_YAW_TI");
	global_data.param[RATE_YAW_TI] = &quadrotor.rateController[YAW].Ti;
	strcpy(global_data.param_name[RATE_YAW_TD], "RATE_YAW_TD");
	global_data.param[RATE_YAW_TD] = &quadrotor.rateController[YAW].Td;
	strcpy(global_data.param_name[RATE_YAW_ND], "RATE_YAW_ND");
	global_data.param[RATE_YAW_ND] = &quadrotor.rateController[YAW].Nd;

	strcpy(global_data.param_name[ATTI_ROLL_K],  "ATTI_ROLL_K");
	global_data.param[ATTI_ROLL_K] = &quadrotor.attiController[ROLL].K;
	strcpy(global_data.param_name[ATTI_ROLL_TI], "ATTI_ROLL_TI");
	global_data.param[ATTI_ROLL_TI] = &quadrotor.attiController[ROLL].Ti;
	strcpy(global_data.param_name[ATTI_ROLL_TD], "ATTI_ROLL_TD");
	global_data.param[ATTI_ROLL_TD] = &quadrotor.attiController[ROLL].Td;
	strcpy(global_data.param_name[ATTI_ROLL_ND], "ATTI_ROLL_ND");
	global_data.param[ATTI_ROLL_ND] = &quadrotor.attiController[ROLL].Nd;

	strcpy(global_data.param_name[ATTI_PITCH_K],  "ATTI_PITCH_K");
	global_data.param[ATTI_PITCH_K] = &quadrotor.attiController[PITCH].K;
	strcpy(global_data.param_name[ATTI_PITCH_TI], "ATTI_PITCH_TI");
	global_data.param[ATTI_PITCH_TI] = &quadrotor.attiController[PITCH].Ti;
	strcpy(global_data.param_name[ATTI_PITCH_TD], "ATTI_PITCH_TD");
	global_data.param[ATTI_PITCH_TD] = &quadrotor.attiController[PITCH].Td;
	strcpy(global_data.param_name[ATTI_PITCH_ND], "ATTI_PITCH_ND");
	global_data.param[ATTI_PITCH_ND] = &quadrotor.attiController[PITCH].Nd;

	strcpy(global_data.param_name[ATTI_YAW_K], "ATTI_YAW_K");
	global_data.param[ATTI_YAW_K] = &quadrotor.attiController[YAW].K;
	strcpy(global_data.param_name[ATTI_YAW_TI], "ATTI_YAW_TI");
	global_data.param[ATTI_YAW_TI] = &quadrotor.attiController[YAW].Ti;
	strcpy(global_data.param_name[ATTI_YAW_TD], "ATTI_YAW_TD");
	global_data.param[ATTI_YAW_TD] = &quadrotor.attiController[YAW].Td;
	strcpy(global_data.param_name[ATTI_YAW_ND], "ATTI_YAW_ND");
	global_data.param[ATTI_YAW_ND] = &quadrotor.attiController[YAW].Nd;

	strcpy(global_data.param_name[ALTI_K], "ALTI_K");
	global_data.param[ALTI_K] = &quadrotor.altitudeController.K;
	strcpy(global_data.param_name[ALTI_TI], "ALTI_TI");
	global_data.param[ALTI_TI] = &quadrotor.altitudeController.Ti;
	strcpy(global_data.param_name[ALTI_TD], "ALTI_TD");
	global_data.param[ALTI_TD] = &quadrotor.altitudeController.Td;
	strcpy(global_data.param_name[ALTI_ND], "ALTI_ND");
	global_data.param[ALTI_ND] = &quadrotor.altitudeController.Nd;

}
