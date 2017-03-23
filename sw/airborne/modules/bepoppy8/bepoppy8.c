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


#if DEBUGGING
#define logTelemetry(msg)	bepoppy8_logTelemetry(msg, (int) strlen(msg));
#else
#define logTelemetry(...)
#endif

struct video_listener *listener = NULL;

// Declare functions - why does this not work when using the header file?
extern uint8_t SearchFloor(struct ClusterInfo);
extern uint8_t ClusterDominance(struct ClusterInfo);
extern uint8_t SafeToGoForwards(uint8_t FloorCluster, uint8_t WindowCluster);
extern uint8_t EscapeLeft(uint8_t WindowDominance, struct ClusterInfo Cluster);

void bepoppy8_init() {
	printf("[bepoppy8_init()] Start\n");
	listener = cv_add_to_device(&front_camera, vision_func); // Initialize listener video_stream
	WindowHalfSize 				= 40;
	deviate 					= 20.0;
	ForwardShift				= 0.2;

	printf("[bepoppy8_init()] Finished\n");
}

void bepoppy8_periodic() {
	// Periodic function that processes the video and decides on the action to take.
	printf("[bepoppy8_periodic()] Start\n");
	uint8_t FloorCluster = SearchFloor(Environment);
	uint8_t DominantWindowCluster = ClusterDominance(Environment);
	uint8_t safe = SafeToGoForwards(FloorCluster, DominantWindowCluster);

	if(safe){
		 bepoppy8_moveWaypointForward(WP_GOAL, ForwardShift);
	}
	else
	{
		bepoppy8_resetWaypoint(WP_GOAL);

		uint8_t goLeft = EscapeLeft(DominantWindowCluster, Environment);

		if(goLeft){
			bepoppy8_AdjustWaypointBearing(WP_GOAL, ForwardShift, -deviate);
			increase_nav_heading(&nav_heading, -deviate);
		}
		else{
			bepoppy8_AdjustWaypointBearing(WP_GOAL, ForwardShift, deviate);
			increase_nav_heading(&nav_heading, deviate);
		}
	}
	printf("[bepoppy8_periodic()] Finished\n");
}

uint8_t SearchFloor(struct ClusterInfo Clusters)
{
	uint8_t RefCluster = 0;

	if(Clusters.Cl2Global>=Clusters.Cl1Global)
	{
		RefCluster = 1;
	}
	else if(Clusters.Cl3Global>=Clusters.Cl2Global)
	{
		RefCluster = 2;
	}
	return RefCluster;
}


uint8_t ClusterDominance(struct ClusterInfo Clusters)
{
	uint8_t WindowDominance = 0;

	if( (Clusters.Cl2AvoidLeft+Clusters.Cl2AvoidRight) >= (Clusters.Cl1AvoidLeft+Clusters.Cl1AvoidRight) )
	{
		WindowDominance = 1;
	}
	else if( (Clusters.Cl3AvoidLeft+Clusters.Cl3AvoidRight) >= (Clusters.Cl2AvoidLeft+Clusters.Cl2AvoidRight) )
	{
		WindowDominance = 2;
	}

	return WindowDominance;
}

uint8_t SafeToGoForwards(uint8_t FloorCluster, uint8_t WindowCluster)
{
if(WindowCluster == FloorCluster)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t EscapeLeft(uint8_t WindowDominance, struct ClusterInfo Cluster)
{
	uint8_t goLeft;
	if(WindowDominance == 0)
	{
		if(Cluster.Cl1AvoidLeft > Cluster.Cl1AvoidRight){goLeft=1;}
		else goLeft=0;
	}
	else if(WindowDominance == 1)
	{
		if(Cluster.Cl2AvoidLeft > Cluster.Cl2AvoidRight){goLeft=1;}
		else goLeft=0;
	}
	else
	{
		if(Cluster.Cl3AvoidLeft > Cluster.Cl3AvoidRight){goLeft=1;}
		else goLeft=0;
	}
	return goLeft;
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
	if (DEBUGGING){
		//DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, nb_msg,  msg);
		printf("%s", msg);
	}
}

void bepoppy8_start(uint8_t waypoint){
	char *msg;
	logTelemetry("bepoppy8_start initiated");

	struct EnuCoor_i shift;
	shift.x 	= POS_BFP_OF_REAL(1.0);
	shift.y 	= POS_BFP_OF_REAL(1.0);

	asprintf(&msg, "Shift: x = %f, y = %f", POS_FLOAT_OF_BFP(shift.x), POS_FLOAT_OF_BFP(shift.y));
	logTelemetry(msg);

	logTelemetry("call moveWaypointTo");
	//bepoppy8_moveWaypointTo(waypoint, 2.5);
}

/*
 * Move the current waypoint with the distances defined by *shift.
 *
 * Tested: By Dave, Tijmen, Joost 14-03-2017
 */
void bepoppy8_moveWaypointBy(uint8_t waypoint, struct EnuCoor_i *shift){
	char *msg; 																// Placeholder Telemetry String
	struct EnuCoor_i new_coor;												// New Coordinate Struct

	struct EnuCoor_i *pos 				= stateGetPositionEnu_i(); 			// Calculate new position of waypoint
	new_coor.x  						= pos->x + shift->x;
	new_coor.y 							= pos->y + shift->y;

	// Telemetry:
	logTelemetry("[bepoppy8] moveWaypointBy:");
	asprintf(&msg, "Current pos: %f, %f", POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y));
	logTelemetry(msg);
	asprintf(&msg, "New pos: %f, %f", POS_FLOAT_OF_BFP(new_coor.x), POS_FLOAT_OF_BFP(new_coor.y));
	logTelemetry(msg);
	asprintf(&msg, "Current Heading: %f", ANGLE_FLOAT_OF_BFP(nav_heading));
	logTelemetry(msg);
	asprintf(&msg, "Desired Heading: %f\n", calculateHeading(shift));
	logTelemetry(msg);

	coordinateTurn(shift); 									// Turn toward new waypoint location.
	waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y); 	// Set x,y position of waypoint

}

/*
 * Move the current waypoint to the location defined by *new_coor.
 *
 * Not tested
 */
void bepoppy8_moveWaypointTo(uint8_t waypoint, struct EnuCoor_i *new_coor){
	struct EnuCoor_i *pos 				= stateGetPositionEnu_i();

	struct EnuCoor_i shift;
	shift.x 							= pos->x - new_coor->x;
	shift.y 							= pos->y - new_coor->y;

	coordinateTurn(&shift);  								// Turn toward new waypoint location.
	waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y); 	// Set x,y position of waypoint

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
 *
 * Untested
 */
void bepoppy8_resetWaypoint(uint8_t waypoint){

	struct EnuCoor_i *pos 				= stateGetPositionEnu_i();
	struct Int32Eulers *eulerAngles   	= stateGetNedToBodyEulers_i();
	struct EnuCoor_i shift;
	struct EnuCoor_i *new_coor;
	float distance = 0.5;

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading                 	= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
	float cos_heading                 	= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));

	// Calculate the shift in position where to place the waypoint you want to go to
	shift.x                       		= POS_BFP_OF_REAL(sin_heading * distance);
	shift.y                       		= POS_BFP_OF_REAL(cos_heading * distance);

	new_coor->x = pos->x + shift.x;
	new_coor->y = pos->y + shift.y;

	coordinateTurn(&shift);  // Double check the correct heading (should not be required);
	waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y); 	// Set x,y position of waypoint

}

/*
 * Move waypoint by a certain heading angle relative to the current heading of the drone
 *
 * Untested
 */
void bepoppy8_AdjustWaypointBearing(uint8_t waypoint, float distance, float HeadingDefl){
	struct EnuCoor_i shift;
	struct Int32Eulers *eulerAngles   	= stateGetNedToBodyEulers_i();

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading                 	= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
	float cos_heading                 	= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));

	// Calculate the shift in position where to place the waypoint you want to go to
	shift.x                       		= POS_BFP_OF_REAL( (sin_heading + HeadingDefl) * distance);
	shift.y                       		= POS_BFP_OF_REAL( (cos_heading + HeadingDefl) * distance);

	bepoppy8_moveWaypointBy(waypoint, &shift);
}

/*
 * Set heading based on the direction of the shift vector.
 *
 * Tested by Dave, Tijmen, Joost  14-03-2017
 */
void coordinateTurn(struct EnuCoor_i *shift){ // Set heading based on shift vector
	float heading_f 					= atan2f(POS_FLOAT_OF_BFP(shift->x), POS_FLOAT_OF_BFP(shift->y));
	nav_heading 						= ANGLE_BFP_OF_REAL(heading_f);
}

/*
 * Calculate heading based on the direction of the shift vector.
 *
 * Tested by Dave 14-03-2017
 */
float calculateHeading(struct EnuCoor_i *shift){ // Returns desired heading based on shift vector, in rad.
	float heading_f 					= atan2f(POS_FLOAT_OF_BFP(shift->x), POS_FLOAT_OF_BFP(shift->y));

	return heading_f;
}

uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees){
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
  INT32_ANGLE_NORMALIZE(newHeading);
  *heading = newHeading;
  return false;
}
