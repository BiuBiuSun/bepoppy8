/*
 *
 * This file is part of paparazzi
 *
 */

/*
 * @file "modules/bepoppy8/bepoppy8.h"
 * Object Avoidance in the TU Delft CyberZoo using the onboard sensors of the Parrot Bebop Drone
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "bepoppy8.h"
#include "modules/computer_vision/lib/vision/image.h"

extern struct image_t *vision_func(struct image_t *);

#ifdef __cplusplus
}
#endif

#if DEBUGGING
#define logTelemetry(msg)	bepoppy8_logTelemetry(msg, (int) strlen(msg));
#else
#define logTelemetry(...)
#endif

