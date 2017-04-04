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
#include "modules/bepoppy8/bepoppy8_cv.h"

#define logTelemetry(msg)	bepoppy8_logTelemetry(msg, (int) strlen(msg));

struct video_listener *listener = NULL;

/*
 * Initialize Global Variables
 * 	- Run at startup of module
 */
void bepoppy8_init() {
	listener = cv_add_to_device(&front_camera, vision_func); 							// Initialize listener video_stream

	// Parameters
	STARTED 					= false;
	NumWindows 					= 5; 					// [-]
	ForwardShift				= 1.0;					// [m]
	FOV 						= 100.0; 				// [deg]
	WindowAngle 				= FOV/NumWindows; 		// [deg]
	windowThreshold 			= 30;					// [px]

	// Initialize mutex for thread safe parameter
	pthread_mutex_init(&navWindow_mutex,NULL);
}

/*
 * Periodic Module Function
 * 	- Set Navigation based on vision thread results
 */
void bepoppy8_periodic() {

	if (STARTED) { 																		// Check if navigation is started

		pthread_mutex_lock(&navWindow_mutex);											// Lock Mutex
		{
		HeadingDeflection = NavWindow*WindowAngle*M_PI/180; 							// Calculate turn angle based on best window option
		NavWindow = 0;																	// Reset window
		}
		pthread_mutex_unlock(&navWindow_mutex); 										// Unlock Mutex

		// Set Navigation
		bepoppy8_AdjustWaypointBearing(WP_GOAL, ForwardShift, HeadingDeflection);

	}

}

/*
 * Use logTelemetry("message") to send debugging related strings to the terminal and GCS.
 *
 * Build a string with variable messages using the following structure:
 * char *msg;
 *
 * asprintf(&msg, "Put static string here, with inserted variables like %s", variable)
 * logTelemetry(msg);
 *
 * #Note: 	Due to a bug in the Paparazzi Code, do NOT use delimiters as ) in the telemetry strings.
 *			There may be more delimiters that also cause errors.
 *
 *	Tested by Dave 13-03-2017
 */
void bepoppy8_logTelemetry(char* msg, int nb_msg) {
		DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, nb_msg,  msg);			// Send INFO_MSG to GCS.
		printf("%s", msg);																// Print string in terminal
}

/*
 * Start execution of periodic avoidance function.
 */
void bepoppy8_start(uint8_t waypoint){
	STARTED = true;
}

/*
 * Stop execution of periodic avoidance function.
 */
  void bepoppy8_stop() {
 	STARTED = false;
 }


/*
 * Move the current waypoint with the distances defined by *shift.
 *
 * Tested: By Dave, Tijmen, Joost 14-03-2017
 */
void bepoppy8_moveWaypointBy(uint8_t waypoint, struct EnuCoor_i *shift){
	struct EnuCoor_i new_coor;												// New Coordinate Struct

	struct EnuCoor_i *pos 				= stateGetPositionEnu_i(); 			// Calculate new position of waypoint
	new_coor.x  						= pos->x + shift->x;
	new_coor.y 							= pos->y + shift->y;

	coordinateTurn(shift); 													// Turn toward new waypoint location.
	waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y); 					// Set x,y position of waypoint

}


/*
 * Move waypoint forward relative to the position of the drone
 *
 * Tested Dave, Tijmen, Joost 14-03-2017
 */
void bepoppy8_moveWaypointForward(uint8_t waypoint, float distance){
	struct EnuCoor_i shift;
	struct Int32Eulers *eulerAngles   	= stateGetNedToBodyEulers_i();

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading                 	= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
	float cos_heading                 	= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));

	// Calculate the shift in position where to place the waypoint you want to go to
	shift.x                       		= POS_BFP_OF_REAL(sin_heading * distance);
	shift.y                       		= POS_BFP_OF_REAL(cos_heading * distance);

	bepoppy8_moveWaypointBy(waypoint, &shift);
}

/*
 * Reset the waypoint location  close to the current position of the drone
 */
void bepoppy8_resetWaypoint(uint8_t waypoint){
	bepoppy8_moveWaypointForward(waypoint, 0.5);
}

/*
 * Move waypoint by a certain heading angle [rad] relative to the current heading of the drone
 */
void bepoppy8_AdjustWaypointBearing(uint8_t waypoint, float distance, float HeadingDefl){
	struct EnuCoor_i shift;
	struct Int32Eulers *eulerAngles   	= stateGetNedToBodyEulers_i();

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading                 	= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + HeadingDefl);
	float cos_heading                 	= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + HeadingDefl);

	// Calculate the shift in position where to place the waypoint you want to go to
	shift.x                       		= POS_BFP_OF_REAL(sin_heading*distance);
	shift.y                       		= POS_BFP_OF_REAL(cos_heading*distance);

	bepoppy8_moveWaypointBy(waypoint, &shift);
}

/*
 * Set heading based on the direction to new waypoint.
 * 	- To avoid to aggressive yaw motion, REF_ERR_R is set to 100.0.
 *
 * Tested by Dave, Tijmen, Joost  14-03-2017
 */
void coordinateTurn(struct EnuCoor_i *shift){
	float heading_f 					= atan2f(POS_FLOAT_OF_BFP(shift->x), POS_FLOAT_OF_BFP(shift->y));		// Calculate new heading
	nav_heading 						= ANGLE_BFP_OF_REAL(heading_f);											// Set heading
}

/*
 * Turn to get back in CyberZoo
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees){
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
  INT32_ANGLE_NORMALIZE(newHeading);
  *heading = newHeading;
  return false;
}
