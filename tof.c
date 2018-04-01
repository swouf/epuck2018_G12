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

static VL53L0X_Dev_t device;

void tof_init(void)
{
	VL53L0X_init(&device);
	VL53L0X_start();

	VL53L0X_configAccuracy(&device, VL53L0X_DEFAULT_MODE);
	VL53L0X_startMeasure(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
}

uint16_t get_distance_tof(void)
{
	return VL53L0X_get_dist_mm();
}
void tof_stop(void)
{
	VL53L0X_stop();
}
