/**
 * \file    tof.c
 * \brief   tof sensor wrapper
 * \author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "tof.h"

//static VL53L0X_Dev_t device;

void tof_init(void)
{
	VL53L0X_start();
}

uint16_t tof_get_distance(void)
{
	uint16_t tof_measured = VL53L0X_get_dist_mm();
	uint16_t tof_corrected = (uint16_t)((0.684*tof_measured-0.6604)+(EPUCK_CIRC/2)+(BALL_SIZE/2)); //correction found experimentally, detailed in the report
	return tof_corrected;
}
void tof_stop(void)
{
	VL53L0X_stop();
}
uint16_t tof_get_ball_pixel_width(uint16_t distance)
{
	uint16_t tof_pixelwidth_ball = 0;
	tof_pixelwidth_ball = LENS_DIAMETER_IN_PIXELS*BALL_SIZE/(2*distance);
	return tof_pixelwidth_ball;
}
