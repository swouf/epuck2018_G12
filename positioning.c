/**
 * \file    positioning.c
 * \brief   E-puck position
 * \date	april 2018
 * \author	J�r�my Jayet (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"
#include "odometric_controller.h"
#include <football.h>
#include <positioning.h>

position_t posGetPos(uint16_t north, uint16_t south, uint16_t west, uint16_t east)
{
	uint16_t arena_length_x = west + east;
	uint16_t arena_length_y = north + south;
	position_t epuck_position;
	epuck_position.x = 1000*west; //converting mm to um by multiplying by 1000
	epuck_position.y = 1000*(south - arena_length_y/2);
	epuck_position.orientation = 0;
	return epuck_position;
}

