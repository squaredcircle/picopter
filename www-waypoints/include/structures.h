#ifndef __COORD_H_GUARD
#define __COORD_H_GUARD

/* Constants */
#define PI					3.14159265359
#define RADIUS_OF_EARTH		6364.963	//km

/* Definitions */
#define sin2(x)				(sin(x)*sin(x))

/* Variables */
#define SPEED_LIMIT				40
#define WAYPOINT_RADIUS			3		//2m;
#define DIRECTION_TEST_DURATION	6000
#define Kp						8		//proportional controller constant/* Constants */
#define PI					3.14159265359
#define RADIUS_OF_EARTH		6364.963	//km

/* Definitions */
#define sin2(x)				(sin(x)*sin(x))

/* Variables */
#define SPEED_LIMIT				40
#define WAYPOINT_RADIUS			3		//2m;
#define DIRECTION_TEST_SPEED	30
#define DIRECTION_TEST_DURATION	6000
#define Kp						8		//proportional controller constant
#define WAIT_AT_WAYPOINTS		3000
#define MAIN_LOOP_DELAY			20


typedef struct {		// Holds a (latitude,longtitude) pair, in radians.
	double lat;
	double lon;
} Coord_rad;

extern FB_Data stop;
extern FB_Data forwards;

#endif