#ifndef __PICOPTER_STRUCTURES_H_GUARD
#define __PICOPTER_STRUCTURES_H_GUARD

/* Constants */
#define PI					3.14159265359
#define RADIUS_OF_EARTH		6364.963	//km
#define RADTODEG(x) 		((x) * (180.0 / PI))
#define DEGTORAD(x)			((x) * (PI / 180.0))

/* Definitions */
#define sin2(x)				(sin(x)*sin(x))

/* Variables */
#define SPEED_LIMIT				40
#define WAYPOINT_RADIUS			3		//2m;
#define DIRECTION_TEST_SPEED	30
#define DIRECTION_TEST_DURATION	6000000
#define Kp						8		//proportional controller constant
#define WAIT_AT_WAYPOINTS		3000000
#define MAIN_LOOP_DELAY			20000


typedef struct {		// Holds a (latitude,longtitude) pair, in degrees.
	double lat;
	double lon;
} coord;

extern FB_Data stop;
extern FB_Data forwards;

#endif