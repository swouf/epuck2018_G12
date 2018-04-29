/**
 * \file    football.c
 * \brief	Launches the simulation
 * \author	J�r�my (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"
#include "main.h"
#include "chprintf.h"
#include "odometric_controller.h"
#include "tof.h"
#include "ball_search.h"
#include <football.h>
#include <leds.h>

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void play(void){
		halInit();
	    chSysInit();
	    mpu_init();

	    //starts the serial communication
	    serial_start();
	    //starts the USB communication
	    usb_start();
	    //init ToF
	    tof_init();

	    odCtrlStart();

	    odCtrlSetPosition(EPUCK_X_START, EPUCK_Y_START, EPUCK_ORIENTATION_START);

	    do{

	    	position_t shooting_position;
	    	position_t ball_position;
	    	BSEMAPHORE_DECL(in_shooting_position, TRUE);
	    	BSEMAPHORE_DECL(ball_found, TRUE);
	    	BSEMAPHORE_DECL(in_initial_position, TRUE);

	    	ball_position = ball_get_position();

	    	//ball_position.x = 100000;
	    	//ball_position.y = 50000;
	    	//ball_position.orientation = 0;

	    	if(ball_position.x && ball_position.y)
	    	{

	    		shooting_position = compute_shooting_position(ball_position);

#ifdef _DEBUG
	    		chprintf((BaseSequentialStream *)&SD3, "BALL POSITION: x = %d um, y = %d um, orientation = %f\n", ball_position.x, ball_position.y, ball_position.orientation);
	    		chprintf((BaseSequentialStream *)&SD3, "SHOOTING POSITION: x = %d um, y = %d um, orientation = %f\n", shooting_position.x, shooting_position.y, shooting_position.orientation);
#endif

	    		odCtrlAddPointToPath(shooting_position.x, shooting_position.y, shooting_position.orientation, &in_shooting_position);

	    		// No one undertstands why, but it works
	    		chBSemWait(&in_shooting_position);

#ifdef _DEBUG
	    		chprintf((BaseSequentialStream *)&SD3, "Epuck in shooting position\n");
#endif


	    		uint32_t distance = 0;

	    		distance = ball_get_distance();

	    		odCtrlMoveForward(distance, &ball_found);

#ifdef _DEBUG
	    		chprintf((BaseSequentialStream *)&SD3, "SHOOTING the ball at a distance %d\n", distance);
#endif

	    		chBSemWait(&ball_found);

#ifdef _DEBUG
	    		chprintf((BaseSequentialStream *)&SD3, "Back home ...\n", distance);
#endif
	    	}

	    	set_body_led(1);

	    	odCtrlAddPointToPath(EPUCK_X_START, EPUCK_Y_START, EPUCK_ORIENTATION_START, &in_initial_position);

	    	chBSemWait(&in_initial_position);

	    	set_body_led(0);

	    }while(1);
}
