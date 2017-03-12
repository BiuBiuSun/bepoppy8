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
#include <string.h>

#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/datalink/telemetry.h"
#include "generated/flight_plan.h"
#include "modules/bepoppy8/bepoppy8.h"

#if DEBUGGING
#define logTelemetry(msg)	bepoppy8_logTelemetry(msg, (int) strlen(msg));
#else
#define logTelemetry(...)
#endif

void bepoppy8_init() {
	// Initial values to be defined at start-up of module

}

void bepoppy8_periodic() {
	// Periodic function that processes the video and decides on the action to take.

}

void bepoppy8_logTelemetry(char* msg, int nb_msg) {
	if (DEBUGGING){
		DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, nb_msg,  msg);
		printf("%s", msg);
	}
}

void bepoppy8_moveWaypoint(uint8_t waypoint, struct EnuCoor_i shift){
	struct EnuCoor_i* new_coor;
	struct EnuCoor_i *pos 	= stateGetPositionEnu_i();
	new_coor->x  			= pos->x + POS_BFP_OF_REAL(shift->x);
	new_coor->y 			= pos->y + POS_BFP_OF_REAL(shift->y);

	coordinateTurn(pos, shift);

	if(shift->z != 0){
		new_coor->z 		= pos->z + POS_BFP_OF_REAL(shift->z);
		waypoint_set_enu_i (waypoint, new_coor);
	}
	else{
		waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
	}

}

void coordinateTurn(struct EnuCoor_i* pos, struct EnuCoor_i* shift){

	int32_t newHeading 	= ANGLE_BFP_OF_REAL(atan2(shift->y,shift->x) + M_PI/2);
	INT32_ANGLENORMALIZE(newHeading);

	&nav_heading 	= newHeading;
}


