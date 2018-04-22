/**
 * \file    odometric_controller.c
 * \brief   odometric controller for epuck2
 * \date	april 2018
 * \author	Jérémy Jayet (jeremy.jayet@epfl.ch)
 * \author	Minh Truong (minh.truong@epfl.ch)
 *
 */

#include "ch.h"
#include "hal.h"
#include "arm_math.h"

#include <math.h>
#include <stdbool.h>

#ifdef _DEBUG
#include <chprintf.h>
#endif

#include <main.h>
#include <motors.h>
#include <odometric_controller.h>

static const float stepLength	=	WHEEL_CIRC/2000;

static position_t	position;
static position_t	path[PATH_BUFFER_SIZE];
static position_t*	pathPtr					= path;
static position_t*	target					= path;
static thread_t*	odRotateThreadPtr		= NULL;
static thread_t*	odMoveForwardThreadPtr	= NULL;

static THD_WORKING_AREA(waOdMoveForward, 256);
static THD_FUNCTION(odMoveForward, lengthPtr) {
	int length = *(int*)lengthPtr;
	arm_pid_instance_q31 forwardPID;

	forwardPID.Kp = FORWARD_KP;
	forwardPID.Ki = FORWARD_KI;
	forwardPID.Kd = FORWARD_KD;

	int error					=	LINEAR_ERROR_MAX+1;
	int linearPos				=	0;
	int linearStep				=	0;
	int rightSpeed				=	0;
	int leftSpeed				=	0;
	uint32_t leftMotorPos		=	0;
	//uint32_t rightMotorPos		=	0;

	int leftMotorDispl		=	0;
	//int rightMotorDispl	=	0;

	float cosOrientation		=	arm_cos_f32(position.orientation);
	float sinOrientation		=	arm_sin_f32(position.orientation);

#ifdef _DEBUG_FORWARD
	chprintf((BaseSequentialStream *)&SD3, "FORWARD PID\n ====================\n");
	chprintf((BaseSequentialStream *)&SD3, "cosOrientation = %f \t sinOrientation = %f \n", cosOrientation, sinOrientation);
#endif

	arm_pid_init_q31(&forwardPID,1);

#ifdef _DEBUG_FORWARD
		chprintf((BaseSequentialStream *)&SD3, "length = %d\n", length);
#endif

	while(((error > LINEAR_ERROR_MAX) & (error > 0)) | ((error < LINEAR_ERROR_MAX) & (error < 0)))
	{
		error = length-linearPos;

		q31_t test = arm_pid_q31(&forwardPID, (q31_t)error);

		//rightSpeed = arm_pid_q31(&forwardPID, (q31_t)error);
		rightSpeed = test;
		leftSpeed = rightSpeed;

		leftMotorPos = left_motor_get_pos();
		//rightMotorPos = right_motor_get_pos();

		left_motor_set_speed(leftSpeed);
		right_motor_set_speed(rightSpeed);

#ifdef _DEBUG_FORWARD
		chprintf((BaseSequentialStream *)&SD3, "---------------------------------------\n");
		chprintf((BaseSequentialStream *)&SD3, "error (int) = %d \n error (q31) = %x \n Speed = %d \n test (dec) = %d\n test (hex) = %x\n", (q31_t)error, error, leftSpeed,test,test);
#endif

		chThdSleepMilliseconds(100);

		left_motor_set_speed(0);
		right_motor_set_speed(0);

		leftMotorDispl	=	left_motor_get_pos()-leftMotorPos;
		//rightMotorDispl	=	right_motor_get_pos()-rightMotorPos;

		linearStep = (int)(leftMotorDispl*stepLength*1000);

		linearPos += linearStep;

		position.x += (int) (cosOrientation*linearStep);
		position.y += (int) (sinOrientation*linearStep);

#ifdef _DEBUG_FORWARD
		chprintf((BaseSequentialStream *)&SD3, "linearStep = %d\t linearPos = %d\n", linearStep, linearPos);
#endif
	}

	//position.x += (int) (cosOrientation*linearPos);
	//position.y += (int) (sinOrientation*linearPos);

	odMoveForwardThreadPtr = NULL;
}

static THD_WORKING_AREA(waOdRotate, 256);
static THD_FUNCTION(odRotate, orientationPtr) {
	float orientation = *((float*)orientationPtr);
	arm_pid_instance_f32 rotationalPID;

	rotationalPID.Kp = ROTATIONAL_KP;
	rotationalPID.Ki = ROTATIONAL_KI;
	rotationalPID.Kd = ROTATIONAL_KD;

	float error					=	ORIENTATION_ERROR_MAX+1;
	int rightSpeed				=	0;
	int leftSpeed				=	0;
	uint32_t leftMotorPos		=	0;
	//uint32_t rightMotorPos		=	0;

	int leftMotorDispl			=	0;
	//int rightMotorDispl			=	0;

	arm_pid_init_f32(&rotationalPID,1);

#ifdef _DEBUG_ROTATE
	chprintf((BaseSequentialStream *)&SD3, "target orientation = %f rad\n", orientation);
#endif


	while(((error > ORIENTATION_ERROR_MAX) & (error > 0)) | ((error < ORIENTATION_ERROR_MAX) & (error < 0)))
	{
		error = orientation-position.orientation;

		rightSpeed = (int)arm_pid_f32(&rotationalPID, error);
		leftSpeed = -rightSpeed;

		leftMotorPos = left_motor_get_pos();
		//rightMotorPos = right_motor_get_pos();

		left_motor_set_speed(leftSpeed);
		right_motor_set_speed(rightSpeed);

#ifdef _DEBUG_ROTATE
		chprintf((BaseSequentialStream *)&SD3, "ORIENTATION PID\n ====================\n");
		chprintf((BaseSequentialStream *)&SD3, "error = %f \n leftSpeed = %d \n", error, leftSpeed);
#endif

		chThdSleepMilliseconds(100);

		left_motor_set_speed(0);
		right_motor_set_speed(0);

		leftMotorDispl	=	leftMotorPos-left_motor_get_pos();
		//rightMotorDispl	=	right_motor_get_pos()-rightMotorPos;

		position.orientation += stepLength*leftMotorDispl*2*PI/(EPUCK_CIRC);

#ifdef _DEBUG_ROTATE
		chprintf((BaseSequentialStream *)&SD3, "left motor displ= %d \n", leftMotorDispl);
#endif

		while(position.orientation > 2*PI)
		{
			position.orientation -= 2*PI;
		}
	}
	odRotateThreadPtr = NULL;
#ifdef _DEBUG_ROTATE
	chprintf((BaseSequentialStream *)&SD3, "Final error = %f \n", error);
#endif
}

static THD_WORKING_AREA(waOdometricRegulator, 1024);
static THD_FUNCTION(odometricRegulator, arg) {

#ifdef _DEBUG_PATH
				chprintf((BaseSequentialStream *)&SD3, "ODOMETRIC REGULATOR\n");
#endif

    //inits the motors
    motors_init();
    position.x = 0;
    position.y = 0;
    position.orientation = 0;

    path[0].x = 0;
    path[0].y = 0;
    path[0].orientation = 0;

    float xd						= 	0;
    float yd						= 	0;
    float alpha						=	0;
    float ratio						=	0;
    float length					=	0;

    while(1)
    {
    	if(target != pathPtr)
    	{
				target++;
				if(target > &(path[PATH_BUFFER_SIZE]))
				{
					target = path;
				}
#ifdef _DEBUG_PATH
				//chprintf((BaseSequentialStream *)&SD3, "After : target = 0x%x & pathPtr = 0x%x \n", target, pathPtr);
				chprintf((BaseSequentialStream *)&SD3, "Target :\tx = %d\t y = %d \t orientation = %f \n", target->x, target->y, target->orientation);
				chprintf((BaseSequentialStream *)&SD3, "Position :\tx = %d\t y = %d \t orientation = %f \n", position.x, position.y, position.orientation);
#endif

			xd = (float) (target->x - position.x);
			yd = (float) (target->y - position.y);

#ifdef _DEBUG_PATH
				chprintf((BaseSequentialStream *)&SD3, "xd = %f , yd = %f\n float length^2 = %f\n", xd, yd, (float)((xd*xd)+(yd*yd)));
#endif

			ratio	=	yd/xd;
			//alpha = 2*PI-position.orientation+atan(ratio);
			alpha = atan(ratio);

			if(xd < 0)
			{
				alpha += PI;
			}

#ifdef _DEBUG_PATH
				chprintf((BaseSequentialStream *)&SD3, "ratio = %f\t alpha = %f\n",ratio, alpha);
#endif

			while(alpha>2*PI)
			{
				alpha -= 2*PI;
			}

			odCtrlRotateTo(alpha);
			chThdWait(odRotateThreadPtr);

			float lengthSquared = (xd*xd)+(yd*yd);

			if(arm_sqrt_f32(lengthSquared, &length) == ARM_MATH_SUCCESS)
			{
				odCtrlMoveForward((int)length);
				chThdWait(odMoveForwardThreadPtr);
			}

			if(target->orientation > 0)
			{
				odCtrlRotateTo(target->orientation);
				chThdWait(odRotateThreadPtr);
			}
    	}
    	chThdSleepMilliseconds(500);
    }
}

void odCtrlStart(void)
{
	chThdCreateStatic(waOdometricRegulator, sizeof(waOdometricRegulator), NORMALPRIO, odometricRegulator, NULL);
}

void odCtrlPause(void);

void odCtrlResume(void);

void odCtrlAddPointToPath(int x, int y, float orientation){
	position_t* a = pathPtr+1;

	if(a > &(path[PATH_BUFFER_SIZE]))
	{
		a = path;
	}

	a->x = x;
	a->y = y;
	a->orientation = orientation;

	pathPtr = a;

#ifdef _DEBUG_PATH
	//chSysLock();
				chprintf((BaseSequentialStream *)&SD3, "Point added to path :\n \
x = %d\t y = %d\t , orientation = %f\n", \
						pathPtr->x, pathPtr->y, pathPtr->orientation);
	//chSysUnlock();
#endif
}

void odCtrlSetPosition(int x, int y, float orientation)
{
	position.x = x;
	position.y = y;
	position.orientation = orientation;
}

void odCtrlRotateTo(float orientation)
{
	static float a = 0;
	a = orientation;
	if(odRotateThreadPtr) chThdWait(odRotateThreadPtr);
	odRotateThreadPtr = chThdCreateStatic(waOdRotate, sizeof(waOdRotate), NORMALPRIO, odRotate, &a);
}
void odCtrlRotate(float alpha)
{
	position_t actualPos = odCtrlGetPosition();

	alpha -= actualPos.orientation;

	for(int i = 0;alpha > PI;i++)
	{
		odCtrlAddPointToPath(actualPos.x, actualPos.y, ((i%2) + 1)*PI);
		alpha -= 2*((i%2))*PI;
	}

	odCtrlAddPointToPath(actualPos.x, actualPos.y, alpha);
}
void odCtrlMoveForward(int length)
{
	static int a = 0;
	a = length;
	if(odMoveForwardThreadPtr) chThdWait(odMoveForwardThreadPtr);
	odMoveForwardThreadPtr = chThdCreateStatic(waOdMoveForward, sizeof(waOdMoveForward), NORMALPRIO, odMoveForward, &a);
}


position_t odCtrlGetPosition(void)
{
	return position;
}
void shoot(void)
{
	float cosOrientation		=	arm_cos_f32(position.orientation);
	float sinOrientation		=	arm_sin_f32(position.orientation);
	int			motorDispl = 0;
	uint32_t	motorPos = left_motor_get_pos();

	left_motor_set_speed(MOTOR_SPEED_LIMIT);
	right_motor_set_speed(MOTOR_SPEED_LIMIT);

	chThdSleepMilliseconds(DISTANCE_EPUCK_BALL/(stepLength*MOTOR_SPEED_LIMIT));

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	motorDispl = left_motor_get_pos()-motorPos;

	position.x += (int) (cosOrientation*motorDispl);
	position.y += (int) (sinOrientation*motorDispl);
}
void odCtrlClear(void)
{
	int index = 0;
	target->x				=	position.x;
	target->y				=	position.y;
	target->orientation		=	position.orientation;
	for(int i = 1;i<PATH_BUFFER_SIZE;i++)
	{
		index = target+(i%PATH_BUFFER_SIZE);
		path[index].x = 0;
		path[index].y = 0;
		path[index].orientation = 0;
	}

	pathPtr = target;
}
