/*
 * tof.h
 *
 *  Created on: 1 avr. 2018
 *      Author: Minh
 */

#ifndef TOF_H_
#define TOF_H_

#define BALL_SIZE 50
#define LENS_DIAMETER_IN_PIXELS 772.55f


/*
 * @brief Initialization of the time-of-flight sensor
 * */
void tof_init(void);
/*
 * @brief Computes the distance between the e-puck and the object facing the sensor by using the time-of-flight sensor
 * @return The distance between the e-puck and the object in front of the sensor
 * */
uint16_t tof_get_distance(void);
/*
 * @brief Turns off the time-of-flight sensor
 * */
void tof_stop(void);
/*
 * @brief Computes the theoretical width in pixels of the ball by using the distance measured by the time-of-flight sensor
 * @param The distance between the e-puck and the object in front of the time-of-flight sensor
 * @return The theoretical width in pixels of the ball with this distance
 * */
uint16_t tof_get_ball_pixel_width(uint16_t distance);


#endif /* TOF_H_ */
