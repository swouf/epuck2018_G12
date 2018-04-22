#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define WHEEL_CIRC 125.664f
#define EPUCK_CIRC 166.504f
#define PI 3.1415927410125732421875f

#define DISTANCE_EPUCK_BALL 30000

#define EPUCK_X_START 500000
#define EPUCK_Y_START 0
#define EPUCK_ORIENTATION_START 3.1415927410125732421875f


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
