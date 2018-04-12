/*
 * ballSearch.h
 *
 *  Created on: 3 avr. 2018
 *      Author: minh1
 */

#ifndef BALL_SEARCH_H_
#define BALL_SEARCH_H_

#include <odometric_controller.h>
/*
 * @brief By using the sensors, the function compute the position of the ball
 * @return The position of the ball
 * */
position_t ball_get_position(void);
/*
 * @brief By using the position of the ball, the function computes the shooting position of the e-puck
 * @param The position of the ball
 * @return The shooting position of the e-puck
 * */
position_t compute_shooting_position(position_t ball_position);


#endif /* BALL_SEARCH_H_ */
