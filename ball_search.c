/*
 * ballSearch.c
 *
 *  Created on: 3 avr. 2018
 *      Author: minh1
 */

#include <stdlib.h>
#include "arm_math.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "tof.h"
#include "ball_search.h"
#include <odometric_controller.h>
#include <process_image.h>
#include <audio/play_melody.h>

position_t ball_get_position(void)
{
	BSEMAPHORE_DECL(ball_spotted, TRUE);

	pImSetBallDetectionSemaphore(&ball_spotted);

	pImProcessImageStart();

#ifdef _DEBUG
	chprintf((BaseSequentialStream *)&SD3, "pImProcessImageStart OK !\n");
#endif

	chBSemWait(&ball_spotted);
	//odCtrlClear();

	position_t epuck_position;
	position_t ball_position;
	uint16_t epuck_ball_distance;
	float ball_direction = 0;

	position_t epuck_actual_position = odCtrlGetPosition();


	epuck_ball_distance = tof_get_distance();
	ball_direction = epuck_actual_position.orientation; // @suppress("Field cannot be resolved")

	ball_position.x = epuck_actual_position.x - epuck_ball_distance*arm_cos_f32(PI-ball_direction);
	ball_position.y = epuck_ball_distance*arm_sin_f32(PI-ball_direction) - epuck_actual_position.y;

	//TEST
//		ball_position.x = 10000;
//		ball_position.y = -100000;
//		ball_position.orientation = 0;

	return ball_position;
}
position_t compute_shooting_position(position_t ball_position){
	position_t shooting_position;
	int xb = ball_position.x;
	int yb = ball_position.y;
	float32_t m = yb/xb;

	//Compute the square root of 1+m^2 and store the result in pOut
	float32_t pOut;
	float32_t pIn = 1+m*m;
	arm_sqrt_f32(pIn, &pOut);

	//pIn = 1+m^2 and pOut = sqrt(1+m^2)
	shooting_position.x = (xb+m*yb+DISTANCE_EPUCK_BALL*pOut)/pIn;

	if(shooting_position.x < xb)
		shooting_position.x = (xb+m*yb-DISTANCE_EPUCK_BALL*pOut)/pIn;

	shooting_position.y = m*shooting_position.x;
	shooting_position.orientation = atan(m)+PI;

	return shooting_position;
}
