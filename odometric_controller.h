/**
 * \file    odometric_controller.h
 * \brief   odometric controller for epuck
 * \author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#ifndef ODOMETRIC_CONTROLLER_H_
#define ODOMETRIC_CONTROLLER_H_

//#define _DEBUG_ROTATE
//#define _DEBUG_FORWARD

#define ROTATIONAL_KP	1000
#define ROTATIONAL_KI	1
#define ROTATIONAL_KD	10

#define FORWARD_KP	0x8FFFFFF
#define FORWARD_KI	0xFFFF
#define FORWARD_KD	0

#define ORIENTATION_ERROR_MAX	0.0175f
#define LINEAR_ERROR_MAX		130

#define PATH_BUFFER_SIZE		10

typedef struct position_t{
	int x;
	int y;
	float orientation;
}position_t;

void odCtrlStart(void);

void odCtrlPause(void);

void odCtrlResume(void);

void odCtrlRotate(float alpha);

void odCtrlAddPointToPath(int x, int y, float orientation);

void odCtrlSetPosition(int x, int y, float orientation);

position_t odCtrlGetPosition(void);

void odCtrlMoveForward(int length);

void odCtrlRotateTo(float orientation);

#endif /* ODOMETRIC_CONTROLLER_H_ */