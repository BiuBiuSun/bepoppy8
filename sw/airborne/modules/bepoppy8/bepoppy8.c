/*
 *
 * This file is part of paparazzi
 *
 */

/*
 * @file "modules/bepoppy8/bepoppy8.c"
 * Object Avoidance in the TU Delft CyberZoo using the on-board sensors of the Parrot Bebop Drone
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "modules/bepoppy8/bepoppy8.h"

void bepoppy8_init() {
	// Initial values to be defined at start-up of module

}

void beboppy8_periodic() {
	// Periodic function that processes the video and decides on the action to take.

}

void beboppy8_move_waypoint() {

}

void bepoppy8_datalinkevent() {

}


uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  VERBOSE_PRINT("Increasing heading to %f\n", ANGLE_FLOAT_OF_BFP(*heading) * 180 / M_PI);
  return false;
}
