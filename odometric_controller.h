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
//#define _DEBUG_ODCTRL

#define ROTATIONAL_KP	1000
#define ROTATIONAL_KI	2
#define ROTATIONAL_KD	10

#define FORWARD_KP	0x8FFFFFF
#define FORWARD_KI	0xFFFF
#define FORWARD_KD	0xFFFFFF

//#define FORWARD_KP	1000
//#define FORWARD_KI	1
//#define FORWARD_KD	10

#define ORIENTATION_ERROR_MAX	0.0350f
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

void odCtrlClear(void);
/*
 * @brief Makes the e-puck rotate counterclockwise of a given angle/Fait tourner le e-puck d'un certain angle dans le sens anti-horaire
 * @param The rotation angle in radians/L'angle de rotation en radians
 * */
void odCtrlRotate(float alpha);
/*
 * @brief Makes the e-puck move to a point in the XY plane and makes it rotate to have the orientation desired
 * /Utilise les moteurs de l'e-puck pour le faire se d�placer � une position XY dans le plan et lui donner une orientation d�sir�e.
 * @param The XY target coordinates and the orientation/Les coordonn�es XY et l'orientation d�sir�es
 * */
void odCtrlAddPointToPath(int x, int y, float orientation, binary_semaphore_t* sem);
/*
 * @brief Sets the position of the e-puck in the XY plane and its orientation/R�gle la position et l'orientation de l'e-puck
 * @param The XY coordinates and orientation/Les coordon�es XY et l'orientation
 * */
void odCtrlSetPosition(int x, int y, float orientation);
/*
 * @brief Returns the actual position and the orientation of the e-puck/Retourne la position et l'orientation actuelle de l'e-puck
 * @return A structure with the position and the orientation of the e-puck/La position et l'orientation de l'e-puck
 * */
position_t odCtrlGetPosition(void);
/*
 * @brief Makes the e-puck move forward for a distance/Fait avancer l'e-puck d'une distance donn�e
 * @param The distance the e-puck travels/La distance de parcours
 * */
void odCtrlMoveForward(int length, binary_semaphore_t* sem);

void odCtrlRotateTo(float orientation);
/*
 * @brief Makes the e-puck shoot the ball by moving forward a distance of 7 cm
 * /Fait tirer le e-puck dans la balle en se d�pla�ant d'une distance de 7 cm
 * */
void odCtrlShoot(void);
/*
 * @brief Stops the e-puck
 * */
void odCtrlStopMovement(void);
/*
 * @brief Sets the e-puck max speed/R�gle la vitesse maximale de l'e-puck
 * @param The max speed/La vitesse maximale de l'e-puck
 * */
void odCtrlSetMaxSpeed(int speed);

#endif /* ODOMETRIC_CONTROLLER_H_ */
