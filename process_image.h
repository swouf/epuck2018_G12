/*
 * \file	process_image.h
 *
 *  \date	april 2018
 *  \author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 *  \author Minh Truong (minh.truong@epfl.ch)
 */

#ifndef PROCESS_IMAGE_H_
#define PROCESS_IMAGE_H_

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			30
#define PXTOMM					31400.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			1200
#define MAX_DIFF_BALL_WIDTH		50
#define N_MEAN					4
#define FOCUS_TOLERANCE			100

#define COLOR_REF_R 198
#define COLOR_REF_G 56
#define COLOR_REF_B 1

typedef struct{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} pixel_t;
/*
 * @brief Initialization of the camera of the e-puck/Initialisation de la cam�ra de l'e-puck
 * */
void pImProcessImageStart(void);
/*
 * @return The horizontal position of the line/La position en X de la ligne
 * */
uint16_t pImGetLinePosition(void);
/*
 * @brief Setting a semaphore when the ball is detected/Mets un s�maphore � 1 quand la balle est d�tect�e
 * @param Semaphore address/L'adresse du s�maphore
 * */
void pImSetBallDetectionSemaphore(binary_semaphore_t* sem);

/*
 * @brief Mesure la distance entre la balle et l'epuck à l'aide de la caméra
 * @return The distance between the e-puck and the object in front of the camera/La distance séparant l'e-puck et l'objet devant lui
 * */
uint16_t pIm_get_distance(void);

#endif /* PROCESS_IMAGE_H_ */
