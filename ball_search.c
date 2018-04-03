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
	odCtrlAddPointToPath(0, 0, PI);
//	pImProcessImageStart();
	chBSemWait(&ball_spotted);


//	play_note(NOTE_A4, 100);
	position_t epuck_position;
	position_t ball_position;
	uint16_t epuck_ball_distance;
	float ball_direction = 0;

	epuck_position = odCtrlGetPosition();
	epuck_ball_distance = tof_get_distance();
	ball_direction = epuck_position.orientation; // @suppress("Field cannot be resolved")

	ball_position.x = epuck_ball_distance*arm_cos_f32(ball_direction);
	ball_position.y = epuck_ball_distance*arm_sin_f32(ball_direction);
	ball_position.orientation = 0;

	return ball_position;
}
position_t compute_shooting_position(position_t ball_position){
	position_t shooting_position;

	int xb = ball_position.x;
	int yb = ball_position.y;
	int m = yb/xb;
	int sqrt_delta = sqrt(yb^2*(2+3*m^2)+DISTANCE_EPUCK_BALL^2*(1+m^2));

	shooting_position.x = (xb+2*m*yb+sqrt_delta)/(1+m^2);
	if(shooting_position.x < xb)
		shooting_position.x = (xb+2*m*yb-sqrt_delta)/(1+m^2);

	shooting_position.y = m*shooting_position.x;
	shooting_position.orientation = atan(m)+PI;

	return shooting_position;
}
