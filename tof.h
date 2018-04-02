/*
 * tof.h
 *
 *  Created on: 1 avr. 2018
 *      Author: Minh
 */

#ifndef TOF_H_
#define TOF_H_

#define BALL_SIZE 5
#define LENS_DIAMETER_IN_PIXELS 772.25f

void tof_init(void);
uint16_t tof_get_distance(void);
void tof_stop(void);
int tof_get_ball_pixel_width(uint16_t tof_get_distance);


#endif /* TOF_H_ */
