/*
 * tof.c
 *
 *  Created on: 1 avr. 2018
 *      Author: Minh
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
	//VL53L0X_init(&device);
	VL53L0X_start();
//	VL53L0X_configAccuracy(&device, VL53L0X_DEFAULT_MODE);
//	VL53L0X_startMeasure(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
}

uint16_t tof_get_distance(void)
{
	return VL53L0X_get_dist_mm();
}
void tof_stop(void)
{
	VL53L0X_stop();
}
int tof_get_ball_pixel_width(uint16_t distance)
{
	uint16_t tof_pixelwidth_ball = 0;
	tof_pixelwidth_ball = LENS_DIAMETER_IN_PIXELS*BALL_SIZE/(2*distance);
	return tof_pixelwidth_ball;
}
