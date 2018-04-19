/*
 * tof.h
 *
 *  Created on: 1 avr. 2018
 *      Author: Minh
 */

#ifndef TOF_H_
#define TOF_H_

#define BALL_SIZE 66
#define LENS_DIAMETER_IN_PIXELS 1445

void tof_init(void);
uint16_t tof_get_distance(void);
void tof_stop(void);
uint16_t tof_get_ball_pixel_width(uint16_t distance);


#endif /* TOF_H_ */
