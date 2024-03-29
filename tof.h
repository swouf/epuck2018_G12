/**
 * \file    tof.h
 * \brief   tof sensor wrapper
 * \author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#ifndef TOF_H_
#define TOF_H_

#define BALL_SIZE 40

/*
 * @brief Initialization of the time-of-flight sensor/Initialise le capteur time-of-flight
 * */
void tof_init(void);

/*
 * @brief Measure (no correction) the distance between the e-puck and the object facing the sensor by using the time-of-flight sensor/
 * Mesure (pas de correction) la distance séparant l'e-puck de l'objet en face du capteur
 * @return The distance between the e-puck and the object in front of the sensor/La distance séparant l'e-puck et l'objet devant lui
 * */
uint16_t tof_get_real_distance(void);

/*
 * @brief Computes the distance between the e-puck and the object facing the sensor by using the time-of-flight sensor/
 * Calcule la distance séparant l'e-puck de l'objet en face du capteur
 * @return The distance between the e-puck and the object in front of the sensor/La distance s�parant l'e-puck et l'objet devant lui
 * */
uint16_t tof_get_distance(void);

/*
 * @brief Turns off the time-of-flight sensor/Eteint le capteur time-of-flight
 * */
void tof_stop(void);

/*
 * @brief Computes the theoretical width in pixels of the ball by using the distance measured by the time-of-flight sensor/
 * Calcule la largeur en pixels th�orique de la balle en fonction de la distance mesur�e par le capteur time-of-flight
 * @param The distance between the e-puck and the object in front of the time-of-flight sensor/La distance mesur�e par le capteur
 * @return The theoretical width in pixels of the ball with this distance/La largeur th�orique en pixels de la balle
 * */
uint16_t tof_get_ball_pixel_width(uint16_t distance);


#endif /* TOF_H_ */
