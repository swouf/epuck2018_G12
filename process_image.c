/**
 * \file	process_image.c
 *
 *  \date	april 2018
 *  \author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 *  \author Minh Truong (minh.truong@epfl.ch)
 */

/**********		INCLUDE		**********/
#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <leds.h>

#include <usbcfg.h>

#include <arm_math.h>

#include <main.h>

#include <camera/dcmi_camera.h>
#include <camera/po8030.h>

#include <process_image.h>
#include <tof.h>
#include <odometric_controller.h>

/**********		STATIC AND CST DECLARATIONS		**********/

static uint16_t	ballWidth		= 0;
static uint16_t	line_position	= IMAGE_BUFFER_SIZE/2;	//middle
static pIm_MODE_t processMode	= SEARCH_BALL;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static binary_semaphore_t* ball_detected = NULL;

// Functions prototypes
static uint16_t pImExtractLineWidth(uint8_t *buffer);
static void pImExtractGreen(uint16_t* input, uint8_t* output, unsigned int size);
static uint32_t pImCompareColors(unsigned int color, unsigned int colorRef);

/**********		THREADS		**********/

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

#ifdef _DEBUG
	chprintf((BaseSequentialStream *)&SD3, "Launching CaptureImage ! \n");
#endif

	dcmi_start();
	chThdSleepMilliseconds(500);
	po8030_start();
	chThdSleepMilliseconds(500);

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 300, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_awb(0);
	po8030_set_rgb_gain(0, 0x48, 0);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);

	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

#ifdef _DEBUG
	chprintf((BaseSequentialStream *)&SD3, " Launching ProcessImage ! \n");
#endif

	uint8_t		*img_buff_ptr;
	uint8_t		image[IMAGE_BUFFER_SIZE] = {0};
	position_t	imagePos;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

        imagePos = odCtrlGetPosition();

		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		/*
		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}
		*/

#ifdef _DEBUG
		static int t = 1;
		if(t)
		{
		uint16_t testColor[5] = {0x18E6, 0x18E8, 0x6332, 0x426E, 0x855F2F};
		uint8_t	testViolet[5] = {0};
		pImExtractViolet(&testColor, &testViolet, 5);

		for(int i = 0; i<5;i++)
		{
			chprintf((BaseSequentialStream *)&SD3, "TestViolet = %x\n", testViolet[i]);
		}
		t=0;
		}
#endif

		//pImExtractViolet((uint16_t*)img_buff_ptr, image, IMAGE_BUFFER_SIZE);
		pImExtractGreen((uint16_t*)img_buff_ptr, image, IMAGE_BUFFER_SIZE);
		//
		SendUint8ToMatlab(image, IMAGE_BUFFER_SIZE);

		if(processMode == SEARCH_BALL)
		{
			//search for a line in the image and gets its width in pixels
			ballWidth					= pImExtractLineWidth(image);
			uint16_t distance			= tof_get_distance();
			uint16_t expectedBallWidth	= tof_get_ball_pixel_width(distance);

			if((abs(ballWidth - expectedBallWidth) < MAX_DIFF_BALL_WIDTH) & (distance < MAX_DISTANCE))
			{
				//pImSetMode(FOCUS_ON_BALL);
#ifdef _DEBUG
				chprintf((BaseSequentialStream *)&SD3, "Color : %x\n", img_buff_ptr[line_position]);
				chprintf((BaseSequentialStream *)&SD3, "ballWidth = %d \t expectedBallWidth = %d\t distance = %d \n", ballWidth, expectedBallWidth, distance);
				chprintf((BaseSequentialStream *)&SD3, "Ball Detected !!!\n");
#endif
				//chBSemSignal(ball_detected);
			}
		}
    }
}

/**********		FUNCTIONS		**********/

/**
 *  @return Returns the line's width extracted from the image buffer given
 *  		Returns 0 if line not found
 */

void SendUint8ToMatlab(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SDU1, (uint8_t*)"S", 1);
	chSequentialStreamWrite((BaseSequentialStream *)&SDU1, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SDU1, (uint8_t*)data, size);
	chSequentialStreamWrite((BaseSequentialStream *)&SDU1, (uint8_t*)"EOF\0", 4);
}

/**
 * @brief Extract the width of a low intensity line in the image.
 *
 * @return line width
 */
static uint16_t pImExtractLineWidth(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//static uint16_t last_width = PXTOCM/GOAL_DISTANCE;
	static uint16_t last_width = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] < mean && buffer[i-WIDTH_SLOPE] > mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line to small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		//width = last_width;
		width = 0;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

//	//sets a maximum width or returns the measured width
//	if((PXTOCM/width) > MAX_DISTANCE){
//		return PXTOCM/MAX_DISTANCE;
//	}else{
//		return width;
//	}
	return width;
}
uint16_t pImGetLinePosition(void){
	return line_position;
}


void pImProcessImageStart(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
#ifdef _DEBUG
	//chprintf((BaseSequentialStream *)&SD3, "ProcessImage launched !\nCaptureImage launched ! \n");
#endif
}
void pImSetBallDetectionSemaphore(binary_semaphore_t* sem){
	ball_detected = sem;
}
void pImSetMode(pIm_MODE_t mode)
{
	processMode = mode;
}
#define MAX_COLOR_ERROR 50
void pImExtractViolet(uint16_t* input, uint8_t* output, unsigned int size)
{
	//unsigned int basisChangeCoeff[3] = {8837, 0 , 9102};
	unsigned int temp = 0;

	unsigned int RGB = 0;
	//unsigned int G = 0;
	//unsigned int B = 0;

	for(int i = 0;i<size;i++)
	{
		RGB =	((input[i]&0xF800)<<5) +\
				((input[i]&0x7E0)<<3) +\
				((input[i]&0x1F));

		chprintf((BaseSequentialStream *)&SD3, "cosAlpha = %d\n",pImCompareColors(RGB, 0x5E6291));

		if(pImCompareColors(RGB, 0x5E6291) < 16384)
		{
			//chprintf((BaseSequentialStream *)&SD3, "R = %d\t G = %d\t B = %d\n", R, G, B);
			output[i] = 0xFF;
		}
		else
		{
			output[i] = 0;
		}
	}
}
static void pImExtractGreen(uint16_t* input, uint8_t* output, unsigned int size)
{
	//unsigned int R = 0;
	unsigned int G = 0;
	//unsigned int B = 0;

	for(int i = 0;i<size;i++)
	{
		//R = ((input[i]&0xF800)>>1);
		G = ((input[i]&0x7E0)<<5);
		//B = ((input[i]&0x1F)<<9);

		output[i] = 0xFF-(G>>10);
	}
}
static uint32_t pImCompareColors(unsigned int color, unsigned int colorRef)
{
	static uint32_t lastColorRef	= 0;

	static uint32_t magnColorRef	=	0;

	uint8_t *colorRefPtr	=	(&colorRef)+1;
	uint8_t *colorPtr	=	(&color)+1;
	q31_t temp			=	0;
	uint32_t magnColor	=	0;
	uint32_t dotProduct	=	0;
	uint32_t cosAlpha	=	0;


	// Si la couleur est la même qu'au dernier appel, ne calcule pas sa norme
	if(lastColorRef != colorRef)
	{
		// CALCUL DE LA NORME

		// Somme des carrés (à optimiser)
		magnColorRef	=	colorRefPtr[0]*colorRefPtr[0] +\
							colorRefPtr[1]*colorRefPtr[1] +\
							colorRefPtr[2]*colorRefPtr[2];

		// Division par 2^18
		temp = magnColorRef<<13;

		arm_sqrt_q31(temp,&temp);

		// Multiplication par 2^9 pour obtenir un Q16.16
		magnColorRef	=	temp>>6;

		lastColorRef = magnColorRef;
	}

	// CALCUL DE LA NORME

	// Somme des carrés (à optimiser)
	magnColor	=	colorPtr[0]*colorPtr[0] +\
			colorPtr[1]*colorPtr[1] +\
			colorPtr[2]*colorPtr[2];

	// Division par 2^18
	temp = magnColor<<13;

	arm_sqrt_q31(temp,&temp);

	// Multiplication par 2^9 pour obtenir un Q16.16
	magnColor	=	temp>>6;

	// CALCUL DU PRODUIT SCALAIRE

	dotProduct =	colorRefPtr[0]*colorPtr[0] +\
					colorRefPtr[1]*colorPtr[1] +\
					colorRefPtr[2]*colorPtr[2];

	dotProduct =	dotProduct<<16;

	cosAlpha	=	dotProduct/(magnColorRef*magnColor);

	return cosAlpha;

}
