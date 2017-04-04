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

// Define switches:
#define VISUALIZE false 				// True = Edit video stream, to visualize vision segmentation
#define useLab true 					// True = Use CIE Lab colorspace instead of YUV422 colorspace

// Initialize Global Variables
bool STARTED;
float ForwardShift;
float FOV;
float WindowAngle;
float HeadingDeflection;
int windowThreshold;

// Initialize Mutex
pthread_mutex_t navWindow_mutex;

// Module Prototype Functions
extern void bepoppy8_init(void); 										// Module init function
extern void bepoppy8_periodic(void);									// Module periodic fuction

extern void bepoppy8_start(uint8_t);									// Start navigation
extern void bepoppy8_stop(void); 										// Stop navigation
extern void bepoppy8_logTelemetry(char*, int); 							// Send telemetry via INFO_MSG and terminal

// Navigation Prototype Functions
extern void bepoppy8_moveWaypointBy(uint8_t, struct EnuCoor_i *);
extern void bepoppy8_moveWaypointForward(uint8_t, float);
extern void bepoppy8_resetWaypoint(uint8_t);
extern void bepoppy8_AdjustWaypointBearing(uint8_t, float, float);
extern void coordinateTurn(struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);


#endif

