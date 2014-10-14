#ifndef __NAVIGATION_STRUCTURES_H_INCLUDED__
#define __NAVIGATION_STRUCTURES_H_INCLUDED__


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


#endif// __NAVIGATION_STRUCTURES_H_INCLUDED__
