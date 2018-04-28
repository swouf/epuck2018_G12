/*
 * \file	process_image.h
 *
 *  \date	april 2018
 *  \author	JÃ©rÃ©my Jayet (jeremy.jayet@epfl.ch)
 *  \author Minh Truong (minh.truong@epfl.ch)
 */

#ifndef PROCESS_IMAGE_H_
#define PROCESS_IMAGE_H_

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			30
#define PXTOMM					31400.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			1000
#define MAX_DIFF_BALL_WIDTH		30
#define N_MEAN					4
#define FOCUS_TOLERANCE			10

#define COLOR_REF_R 198
#define COLOR_REF_G 56
#define COLOR_REF_B 1

typedef enum {
	SEARCH_BALL,
	FOCUS_ON_BALL,
} pIm_MODE_t;

typedef struct{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} pixel_t;
/*
 * @brief Initialization of the camera of the e-puck/Initialisation de la caméra de l'e-puck
 * */
void pImProcessImageStart(void);
/*
 * @return The horizontal position of the line/La position en X de la ligne
 * */
uint16_t pImGetLinePosition(void);
/*
 * @brief Setting a semaphore when the ball is detected/Mets un sémaphore à 1 quand la balle est détectée
 * @param Semaphore address/L'adresse du sémaphore
 * */
void pImSetBallDetectionSemaphore(binary_semaphore_t* sem);
/*
 * @brief Computes the position of the e-puck
 * @param The distances between the e-puck and the four walls of the squared arena
 * */
void pImSetMode(pIm_MODE_t mode);
/*
 * @brief Computes the position of the e-puck
 * @param The distances between the e-puck and the four walls of the squared arena
 * */
void pImExtractColor(uint16_t* input, uint8_t* output, unsigned int size);

#endif /* PROCESS_IMAGE_H_ */
