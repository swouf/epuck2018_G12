/**
 * \file    football.h
 * \brief	Launches the simulation
 * \author	Jérémy (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#ifndef FOOTBALL_H_
#define FOOTBALL_H_

#define WHEEL_CIRC 125.664f
#define EPUCK_CIRC 166.504f
#define PI 3.1415927410125732421875f

#define DISTANCE_EPUCK_BALL 70000

#define EPUCK_X_START 500000
#define EPUCK_Y_START 0
#define EPUCK_ORIENTATION_START 3.1415927410125732421875f

static void serial_start(void);
/*
 * @brief Make the e-puck shoot the ball in the target/Le e-puck effectue le tir dans le but
 * */
void play(void);


#endif /* FOOTBALL_H_ */
