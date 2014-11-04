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


#endif// __NAVIGATION_STRUCTURES_H_INCLUDED__
