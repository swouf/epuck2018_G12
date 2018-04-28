/**
 * \file    ball_search.c
 * \brief   Ball position and shooting position
 * \date	april 2018
 * \author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#ifndef BALL_SEARCH_H_
#define BALL_SEARCH_H_

#include <odometric_controller.h>
/*
 * @brief Computes the position of the ball by using the sensor/Calcule la position de la balle en utilisant les données du capteur
 * @return The position of the ball/La position de la balle
 * */
position_t ball_get_position(void);
/*
 * @brief Computes the shooting position of the e-puck by knowing the position of the ball/Calcule la position de tir du e-puck
 * @param The position of the ball/La position de la balle
 * @return The shooting position of the e-puck/La position de tir du e-puck
 * */
position_t compute_shooting_position(position_t ball_position);


#endif /* BALL_SEARCH_H_ */
