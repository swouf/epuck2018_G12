/**
 *  \file	process_image.c
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

static thread_t* CaptureImagePtr	=	NULL;
static thread_t* ProcessImagePtr	=	NULL;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(processImageRun, TRUE);
static binary_semaphore_t* ball_detected = NULL;

// Functions prototypes
static uint16_t pImExtractLineWidth(uint8_t *buffer);
static void pImExtractBlack(uint16_t* input, uint8_t* output, unsigned int size);
static uint8_t pImCompareColors(pixel_t color, pixel_t colorRef);

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

#if N_MEAN == 1
	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 224, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
#else
	po8030_advanced_config(FORMAT_RGB565, 0, 224, IMAGE_BUFFER_SIZE, N_MEAN, SUBSAMPLING_X1, SUBSAMPLING_X1);
#endif
	//po8030_set_awb(0);
	po8030_set_rgb_gain(0x48, 0x48, 0x48);
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

	uint16_t	line_pos_center	=	0;

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
		pImExtractColor(&testColor, &testViolet, 5);

		for(int i = 0; i<5;i++)
		{
			chprintf((BaseSequentialStream *)&SD3, "TestViolet = %x\n", testViolet[i]);
		}
		t=0;
		}
#endif

		//pImExtractColor((uint16_t*)img_buff_ptr, image, IMAGE_BUFFER_SIZE);
		pImExtractBlack((uint16_t*)img_buff_ptr, image, IMAGE_BUFFER_SIZE);

		//SendUint8ToMatlab(image, IMAGE_BUFFER_SIZE);

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
				chBSemSignal(ball_detected);
				break;
			}
		}
		/*
		else if (processMode == FOCUS_ON_BALL)
		{
			line_pos_center = line_position - IMAGE_BUFFER_SIZE/2;
			if(line_pos_center < FOCUS_TOLERANCE)
			{

			}
			else
			{
				int speed = 2200*(line_pos_center/(IMAGE_BUFFER_SIZE/2));

				left_motor_set_speed(speed);
				right_motor_set_speed(-speed);
			}
		}
		*/
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
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		        //chprintf((BaseSequentialStream *)&SD3, "BEGIN FOUND in %d !\n", i);
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    i += 10;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean)
		        {
		            end = i;
		            stop = 1;
		            //chprintf((BaseSequentialStream *)&SD3, "END FOUND in %d !\n", i);
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

	//chprintf((BaseSequentialStream *)&SD3, "begin = %d, end = %d, line_not_found = %d\n", begin, end, line_not_found);

	if(line_not_found){
		begin = 0;
		end = 0;
		//width = last_width;
		width = 0;
	}
	else
	{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//chprintf((BaseSequentialStream *)&SD3, "width = %d\n", width);

	return width;
}
uint16_t pImGetLinePosition(void){
	return line_position;
}


void pImProcessImageStart(void){
	if(ProcessImagePtr == NULL){ProcessImagePtr = chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);}
	if(CaptureImagePtr == NULL){CaptureImagePtr = chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);}
	chBSemSignal(&processImageRun);
}
void pImSetBallDetectionSemaphore(binary_semaphore_t* sem){
	ball_detected = sem;
}
void pImSetMode(pIm_MODE_t mode)
{
	processMode = mode;
}
#define MAX_COLOR_ERROR 50
void pImExtractColor(uint16_t* input, uint8_t* output, unsigned int size)
{
	unsigned int temp = 0;

	unsigned int RGB = 0;
	pixel_t	pixel;
	pixel_t pixelRef;

	pixelRef.r = COLOR_REF_R;
	pixelRef.g = COLOR_REF_G;
	pixelRef.b = COLOR_REF_B;

	for(int i = 0;i<size;i++)
	{
		temp = 0;
		for(int j = 0; j<N_MEAN;j++)
		{
			temp += input[i+j*IMAGE_BUFFER_SIZE];
		}
		input[i] = temp/N_MEAN;

		pixel.b		= ((input[i]&0x1F))*8;
		pixel.r		= ((input[i]&0xF800)>>11)*8;
		pixel.g		= ((input[i]&0x7E0)>>5)*4;

		if(pImCompareColors(pixel, pixelRef) > 240)
		{
			output[i] = 0xFF;
		}
		else
		{
			output[i] = 0;
		}
		//output[i] = pImCompareColors(pixel, pixelRef);
		//output[i] = pixel.r;
	}
}
static void pImExtractBlack(uint16_t* input, uint8_t* output, unsigned int size)
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
void pImTest(uint32_t t)
{
	t = t;
}
static uint8_t pImCompareColors(pixel_t color, pixel_t colorRef)
{
	static pixel_t lastColorRef;

	static uint32_t magnColorRef		=	0;

	q31_t temp							=	0;
	uint32_t magnColor					=	0;
	volatile uint32_t dotProduct		=	0;
	uint32_t cosAlpha					=	0;

	const uint32_t* dotProductPtr = &dotProduct;


	// Si la couleur est la même qu'au dernier appel, ne calcule pas sa norme
	if(lastColorRef.r != colorRef.r)
	{
		// CALCUL DE LA NORME

		// Somme des carrés (à optimiser)
		magnColorRef	=	colorRef.r*colorRef.r +\
							colorRef.g*colorRef.g +\
							colorRef.b*colorRef.b;

		// Division par 2^18
		temp = magnColorRef<<13;

		arm_sqrt_q31(temp,&temp);

		// Multiplication par 2^9 pour obtenir un Q24.8
		magnColorRef	=	temp>>14;

		lastColorRef = colorRef;
	}

	// CALCUL DE LA NORME

	// Somme des carrés (à optimiser)
	magnColor	=	color.r*color.r +\
					color.g*color.g +\
					color.b*color.b;

	// Division par 2^18
	temp = magnColor<<13;

	arm_sqrt_q31(temp,&temp);

	// Multiplication par 2^9 pour obtenir un Q24.8
	magnColor	=	temp>>14;

	// CALCUL DU PRODUIT SCALAIRE

	dotProduct =	colorRef.r*color.r +\
					colorRef.g*color.g +\
					colorRef.b*color.b;

	//cosAlpha	=	(256*((256*dotProduct)/magnColorRef))/magnColor;
	cosAlpha	=	256*dotProduct;
	//chThdSleepSeconds(1);
	cosAlpha	/=	magnColorRef;
	//chThdSleepSeconds(1);
	cosAlpha	*=	65536;
	//chThdSleepSeconds(1);
	cosAlpha	/=magnColor;
	//chThdSleepSeconds(1);

	//pImTest(dotProduct);

	return cosAlpha;
}
uint16_t pIm_get_distance(void)
{
	uint16_t distance = PXTOCM/ballWidth;
	return distance;
}
