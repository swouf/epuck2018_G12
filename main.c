#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <odometric_controller.h>
#include "tof.h"

#define _DEBUG

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

#ifdef _DEBUG

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

#endif

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //init ToF
    tof_init();

#ifdef _DEBUG
    //starts timer 12
    timer12_start();
#endif

    /*float test = 2.0;
    volatile uint16_t time = 0;
    chSysLock(); //reset the timer counter
    GPTD12.tim->CNT = 0;

    for(int i = 0; i<4;i++)
    {
    	test = atan(test);
    }

    time = GPTD12.tim->CNT;
    chSysUnlock();

    float penis = 12.3;
    uint16_t gpenis = (uint16_t) penis;

    while (1) {
    	chprintf((BaseSequentialStream *)&SDU1, "gpenis=%d\n", gpenis);
    	chprintf((BaseSequentialStream *)&SD3, "gpenis=%d\n", gpenis);
    	chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n test = %f\n", time, test);
    	chprintf((BaseSequentialStream *)&SD3, "time=%dus\n test = %f\n", time, test);
    	chThdSleepMilliseconds(1000);

    }*/

#ifdef _DEBUG
				chprintf((BaseSequentialStream *)&SD3, "distance Time Of Flight = %d mm\n", tof_get_distance());
#endif

	odCtrlAddPointToPath(1, 200000, 0);
	odCtrlAddPointToPath(200000, 200000, 0);
	odCtrlAddPointToPath(200000, 1, 0);
	odCtrlAddPointToPath(1, 1, PI/2);

	odCtrlAddPointToPath(200000, 0, 0);
	odCtrlAddPointToPath(200000, 200000, 0);
	odCtrlAddPointToPath(0, 200000, 0);
	odCtrlAddPointToPath(1, 1, PI/2);

//    odCtrlStart();

    while (1) {

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
