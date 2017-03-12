/*
 *
 * This file is part of paparazzi
 *
 */

/*
 * @file "modules/bepoppy8/bepoppy8.h"
 * Object Avoidance in the TU Delft CyberZoo using the onboard sensors of the Parrot Bebop Drone
 */

#ifndef BEPOPPY8_H
#define BEPOPPY8_H
#include <inttypes.h>

#define DEBUGGING true

extern void bepoppy8_init(void);
extern void bepoppy8_periodic(void);

extern void bepoppy8_logTelemetry(char*, int);
extern void bepoppy8_moveWaypoint(uint8_t, struct EnuCoor_i);

#endif

