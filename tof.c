/**
 * \file    tof.c
 * \brief   tof sensor wrapper
 * \author	J�r�my Jayet (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "tof.h"
#include "process_image.h"

//static VL53L0X_Dev_t device;

void tof_init(void)
{
	VL53L0X_start();
}

uint16_t tof_get_distance(void)
{
	uint16_t tof_measured = VL53L0X_get_dist_mm();
	uint16_t tof_corrected = 0;

	if(tof_measured > 230)
	{
		tof_corrected = (uint16_t)((0.66*tof_measured-4.3)+(EPUCK_CIRC/2)+(BALL_SIZE/2));
	}
	else
	{
		tof_corrected = (uint16_t)((0.54*tof_measured-0.26)+(EPUCK_CIRC/2)+(BALL_SIZE/2));
	}

	return tof_corrected;
}
void tof_stop(void)
{
	VL53L0X_stop();
}
uint16_t tof_get_ball_pixel_width(uint16_t distance)
{
	uint16_t tof_pixelwidth_ball = 0;
	if(distance > 190) {tof_pixelwidth_ball = PXTOMM/distance;}
	else {tof_pixelwidth_ball = 1.212*(PXTOMM/distance)+6;}
	return tof_pixelwidth_ball;
}
