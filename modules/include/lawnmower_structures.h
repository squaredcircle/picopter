#ifndef __LAWNMOWER_STRUCTURES_INCLUDED__
#define __LAWNMOWER_STRUCTURES_INCLUDED__

#define LOCATION_WAIT 0		//Time in us Copter waits at each point
#define LOOP_WAIT 100000	//Time in us Copter wait in each loop

#define MAXLAT -31.979422	//Properties of image file of James Oval & represent min & max corners - are in degrees
#define MINLON 115.817162
#define MINLAT -31.980634
#define MAXLON 115.818709
#define PIXEL_RADIUS 1		//Number of surrounding pixels to turn Black. Can probably be left as 0, unless get really fine image.

#define PI 3.14159265359
#define RADIUS_OF_EARTH 6364963				//m
#define sin2(x) (sin(x)*sin(x))
#define DIRECTION_TEST_SPEED 40
#define DIRECTION_TEST_DURATION 6000000		//us
#define PAST_POINTS 10						//Number of past points to save for integral contol

#define OBJECT_LIMIT 5

typedef struct{		//These are in degrees now.
	double lat;
	double lon;
} Pos;

extern bool exitLawnmower;
extern bool usingIMU;
extern bool usingWindows;

extern int SPEED_LIMIT;		//Config file parameters - need to be initialised as globals
extern double SWEEP_SPACING;
extern double POINT_SPACING;
extern double WAYPOINT_RADIUS;
extern double KPxy;
extern double KIxy;
extern double KPz;
extern double KIz;
extern double DURATION;
extern double FREQUENCY;
extern int VOLUME;

extern int HMIN;
extern int HMAX;
extern int SMIN;
extern int SMAX;
extern int VMINIMUM;
extern int VMAX;
extern int WHITE;
extern int BLACK;
extern int COLSIZE;
extern int ROWSIZE; 
extern int PIXELTHRESH;
extern int DILATE_ELEMENT;
extern int ERODE_ELEMENT;

#endif
