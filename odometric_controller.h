/**
 * @file    odometric_controller.h
 * @brief   odometric controller for epuck
 * @author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 *
 */

#ifndef ODOMETRIC_CONTROLLER_H_
#define ODOMETRIC_CONTROLLER_H_

//#define _DEBUG_ROTATE
#define _DEBUG_FORWARD

#define ROTATIONAL_KP	1000
#define ROTATIONAL_KI	1
#define ROTATIONAL_KD	10

#define FORWARD_KP	0xFFFFFF
#define FORWARD_KI	0xFFF
#define FORWARD_KD	0

#define ORIENTATION_ERROR_MAX	0.0175f
#define LINEAR_ERROR_MAX		130

typedef struct position_t{
	int x;
	int y;
	float orientation;
}position_t;

void odCtrlStart(void);

void odCtrlPause(void);

void odCtrlResume(void);

void odCtrlAddPointToPath(int x, int y, float orientation);

void odCtrlSetPosition(int x, int y, float orientation);

position_t* odCtrlGetPosition(void);

#endif /* ODOMETRIC_CONTROLLER_H_ */
