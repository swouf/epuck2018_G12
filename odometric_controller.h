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
//#define _DEBUG_PATH

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

/*
 * @brief Initialize the thread
 * */
void odCtrlStart(void);

void odCtrlPause(void);

void odCtrlResume(void);
/*
 * @brief Makes the e-puck rotate counterclockwise of a given angle
 * @param The rotation angle in radians
 * */
void odCtrlRotate(float alpha);
/*
 * @brief Makes the e-puck move to a point in the XY plane and makes it rotate to have the orientation desired
 * @param The XY coordinates and the orientation
 * */
void odCtrlAddPointToPath(int x, int y, float orientation);
/*
 * @brief Sets the position of the e-puck in the XY plane and its orientation
 * @param The XY coordinates and orientation
 * */
void odCtrlSetPosition(int x, int y, float orientation);
/*
 * @brief Returns the actual position and the orientation of the e-puck
 * @return A structure with the position and the orientation of the e-puck
 * */
position_t odCtrlGetPosition(void);
/*
 * @brief Makes the e-puck to move forward
 * @param The distance which the e-puck travels
 * */
void odCtrlMoveForward(int length);

void odCtrlRotateTo(float orientation);
/*
 * @brief Makes the e-puck shoot the ball by moving forward a distance of 7 cm
 * */
void shoot(void);


#endif /* ODOMETRIC_CONTROLLER_H_ */
