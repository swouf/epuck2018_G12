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
#define MIN_LINE_WIDTH			40
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define MAX_DIFF_BALL_WIDTH		10

void pImProcessImageStart(void);
uint16_t pImGetLinePosition(void);

#endif /* PROCESS_IMAGE_H_ */