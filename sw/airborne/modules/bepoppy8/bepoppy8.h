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
#include "state.h"

#define DEBUGGING true

extern void bepoppy8_init(void);
extern void bepoppy8_periodic(void);
//extern struct image_t *vision_func(struct image_t *);

extern void bepoppy8_start(uint8_t);

extern void bepoppy8_logTelemetry(char*, int);
extern void bepoppy8_moveWaypointBy(uint8_t, struct EnuCoor_i *);
extern void bepoppy8_moveWaypointTo(uint8_t, struct EnuCoor_i *);
extern void bepoppy8_moveWaypointForward(uint8_t, float);
extern void coordinateTurn(struct EnuCoor_i *);
extern float calculateHeading(struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);

#endif

