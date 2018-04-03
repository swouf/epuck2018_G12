/*
 * ballSearch.c
 *
 *  Created on: 3 avr. 2018
 *      Author: minh1
 */

#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "tof.h"
#include "ball_search.h"
#include <odometric_controller.h>

position_t ball_get_position(void)
{
 //robot fait un 360°
	position_t epuck_position;
	position_t ball_position;
	uint16_t epuck_ball_distance;
	float ball_direction = 0;

	epuck_position = odCtrlGetPosition();
	epuck_ball_distance = tof_get_distance();
	ball_direction = epuck_position.orientation;

	ball_position.x = epuck_ball_distance*cos(ball_direction);
	ball_position.y = epuck_ball_distance*sin(ball_direction);
	ball_position.orientation = 0;

	return ball_position;
}

