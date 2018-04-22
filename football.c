/*
 * lib.c
 *
 *  Created on: 19 Apr 2018
 *      Author: truong
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

		position_t shooting_position;
		shooting_position = compute_shooting_position(ball_get_position());
		chprintf((BaseSequentialStream *)&SD3, "SHOOTING POSITION: x = %d um, y = %d um, orientation = %f\n", shooting_position.x, shooting_position.y, shooting_position.orientation);

		odCtrlAddPointToPath(shooting_position.x, shooting_position.y, shooting_position.orientation);

	    chThdSleepMilliseconds(200);

	    while (1) {
	    	chprintf((BaseSequentialStream *)&SD3, "TIME OF FLIGHT DISTANCE = %d mm\n", tof_get_distance());
	    	chThdSleepSeconds(3);
	    }
}
