#ifndef __NAVIGATION_STRUCTURES_H_INCLUDED__
#define __NAVIGATION_STRUCTURES_H_INCLUDED__


/* Constants */
#define PI					3.14159265359
#define RADIUS_OF_EARTH		6364.963	//km

/* Preprocessor functions */
#define RAD2DEG(x) 			((x) * (180.0 / PI))
#define DEG2RAD(x)			((x) * (PI / 180.0))
#define sin2(x)				(sin(x)*sin(x))

#define MIN(x, y)			(((x) < (y)) ? (x) : (y))
#define MAX(x, y)			(((x) > (y)) ? (x) : (y))
#define SIGNUM(x)			(((x) > 0) ? 1 : (((x) < 0) ? -1 : 0))

/* Structures */
typedef struct {		// Holds a (latitude,longtitude) pair, in degrees.
	double lat;
	double lon;
} coord;


typedef struct {		// Holds a (speed, bearing) pair.
	double speed;			//Speed it %power (100 = full speed ahead (warp speed), 50 = half speed, 0 = stop).
	double bearing;			//Direction in degrees right of north.
} velocity;



typedef struct {		// Holds an (x,y) pair, in meters.  May be either a point or a vector.
	double x;
	double y;
} cartesian;

typedef struct {		// Holds an (X1,X2) pair, in meters. And the distance between them.
	cartesian X1;
	cartesian X2;
	coord origin;
} line;



typedef struct {		// Holds an (X1,X2) pair, in meters. And the distance between them.
	bool FB_Working;
	bool GPS_Working;
	bool IMU_Working;
	bool CAM_Working;
} hardware_checks;


#endif// __NAVIGATION_STRUCTURES_H_INCLUDED__
