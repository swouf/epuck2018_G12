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
 * @brief
 * @return
 * */
position_t ball_get_position(void);
/*
 * @brief
 * @param
 * @return
 * */
position_t compute_shooting_position(position_t ball_position);


#endif /* BALL_SEARCH_H_ */
