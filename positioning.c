/*
 * positioning.c
 *
 *  Created on: 28 avr. 2018
 *      Author: minh1
 */
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"
#include "odometric_controller.h"
#include <positioning.h>

position_t posGetPos(uint16_t north, uint16_t south, uint16_t west, uint16_t east)
{
	uint16_t arena_length_x = west + east;
	uint16_t arena_length_y = north + south;
	position_t epuck_position;
	epuck_position.x = 1000*west; //converting mm to um
	epuck_position.y = 1000*(south - arena_length_y/2);
	epuck_position.orientation = 0;
	return epuck_position;
}

